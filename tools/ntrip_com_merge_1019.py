#!/usr/bin/env python3
"""
一键：NTRIP 差分 + 本地广播星历 RINEX → 合并写入同一串口（补齐 RTCM 1019）。

MCU 固件默认按单点解（SPP）输出 $PRTK 经纬度；本脚本侧重「星历 1019 + 下行 RTCM」灌入 USART3，而非 MCU 内 RTK 固定解。

你必须先关掉 STRSVR（或其它占用 COM 的程序），同一 COM 只能被一个进程打开。

当前服务商示例（全国/QX 账号 App；账号 qx4839，caster 114.111.30.20）：
  • 端口 8002 — WGS84（端口号2）
  • 端口 8003 — CGCS2000（端口号1）；脚本 --port 默认即 8003
  • 接入点 RTCM33_GGB；亦可试 接入点2 AUTO（是否可用取决于 caster）

坐标框架只影响 --port（同一 caster 上换端口即换框架）：
  8001 — ITRF2008（历元约 2016）
  8002 — WGS84（历元约 2005）
  8003 — CGCS2000（历元约 2000），脚本默认端口即此项

用法示例（挂载点须与列表大小写完全一致；密码勿写入仓库，用环境变量）：
  pip install pyserial

  设置密码（两种窗口语法不同，勿混用）：
    • CMD（命令提示符，提示符常为 D:\\...>）：set NTRIP_PASSWORD=你的密码
      （set 与变量名、等号、值之间不要插空格。）
    • PowerShell（提示符常为 PS D:\\...>）：$env:NTRIP_PASSWORD='你的密码'

  set NTRIP_PASSWORD=你的密码
  python tools/ntrip_com_merge_1019.py ^
    --rinex brdc1250.26n ^
    --caster 114.111.30.20 --port 8002 ^
    --user qx4839 --password-env NTRIP_PASSWORD ^
    --mount RTCM33_GGB ^
    --com COM7 --baud 115200 --eph-interval 10

  省略 --caster / --user / --mount 时可用脚本内默认值（与本示例一致）。
  本例端口 8002 = WGS84，与常见接收机/RTKLIB 一致；若要用 CGCS2000 则 --port 8003（或不写 --port）。
  其它 caster 上常用挂载示例：RTCM32GRCpro、RTCM33GRCEJpro（须与该 caster 源列表一致）。

密码也可直接用 --password（注意会进命令行历史）。星历文件：仓库根目录默认可用 brdc1250.26n；亦支持 .gz、IGS 当天混合 NAV v3（*MN.rnx）；路径传给 --rinex。

若报错「没有可用的 GPS 星历」：与 COM/CH340 无关；请换当天 brdc，或加 --max-age-hours 0，并核对电脑日期时间。

若启动时报 serial.SerialException / 错误 31「设备未发挥作用」：多为 COM 被占用、--com 选错、USB/驱动问题；见运行时脚本给出的排查列表。
"""

from __future__ import annotations

import argparse
import base64
import pathlib
import queue
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

_TOOLS = pathlib.Path(__file__).resolve().parent
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import brdc_to_rtcm1019 as brdc  # noqa: E402


def _serial_open_diagnostic(com: str, exc: BaseException) -> str:
    """Human-readable hint when pyserial cannot open/configure the port (e.g. WinError 31)."""
    return (
        f"无法打开或配置串口 {com!r}\n"
        f"系统消息: {exc}\n\n"
        "常见原因（含 Windows 错误 31「连到系统上的设备没有发挥作用」、拒绝访问）：\n"
        "  1) COM 被其它程序占用 — 先关掉 SSCOM、STM32CubeIDE「串口终端」、u-center、Arduino 串口监视器等，"
        "再单独运行本脚本。\n"
        "  2) --com 与实际不符 — 在「设备管理器 → 端口(COM 和 LPT)」确认 CH340/CP210x 对应编号；"
        "插拔 USB 后 COM 数字常会变。\n"
        "  3) USB/驱动 — 换数据线、换 USB 口；重装 CH340/CP2102 驱动；设备上黄色叹号需先解决驱动。\n"
        "  4) ST-Link 的虚拟串口 ≠ 接 MCU USART3 的 USB-TTL — NTRIP 数据应打到 CH340 那一 COM，"
        "不要误用 ST-Link 的 COM（除非硬件短接同一路，一般不会）。\n"
        "  5) 若确定没有其它软件占用仍报「拒绝访问」：多为 USB 抖动/驱动或旧版 pyserial 在多线程下同时 write；"
        "请更新 pyserial、换 USB 口/换线，本脚本已改为单线程写串口以降低此类错误。\n"
    )


def _basic_auth(user: str, password: str) -> str:
    token = base64.b64encode(f"{user}:{password}".encode("utf-8")).decode("ascii")
    return f"Basic {token}"


def ntrip_open_stream(
    host: str,
    port: int,
    mountpoint: str,
    user: str,
    password: str,
    timeout: float,
) -> socket.socket:
    """TCP + NTRIP GET；返回已跳过 HTTP/ICY 头的阻塞 socket。"""
    mp = mountpoint.strip()
    if not mp:
        raise ValueError("mountpoint 不能为空（例如 RTCM32GRCpro）")

    # HTTP/1.1 + keep-alive：避免部分 caster 在「Connection: close」下很快结束流传输，
    # 造成 COM 断续、MCU 侧 rx2 停涨 / basN 被空闲逻辑清空。
    req_lines = [
        f"GET /{mp} HTTP/1.1",
        f"Host: {host}",
        "User-Agent: NTRIP ntrip_com_merge_1019",
        "Accept: */*",
        "Connection: keep-alive",
        "Ntrip-Version: Ntrip/1.0",
        f"Authorization: {_basic_auth(user, password)}",
        "",
        "",
    ]
    payload = "\r\n".join(req_lines).encode("ascii")

    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except (AttributeError, OSError):
        pass
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.settimeout(timeout)
    sock.sendall(payload)

    header = bytearray()
    while b"\r\n\r\n" not in header:
        chunk = sock.recv(1)
        if not chunk:
            sock.close()
            raise ConnectionError("NTRIP：连接在收到响应头之前断开")
        header.extend(chunk)
        if len(header) > 65536:
            sock.close()
            raise ConnectionError("NTRIP：响应头过长")

    head_txt = header.decode("latin-1", errors="replace")
    first = head_txt.split("\r\n", 1)[0].strip()
    if "200" not in first:
        sock.close()
        raise ConnectionError(f"NTRIP 失败（检查账号/挂载点/密码）: {first!r}")

    sock.settimeout(300.0)
    return sock


def load_1019_batch(rinex: Path, latest_per_prn: bool, max_age_hours: float) -> bytes:
    rinex_path = rinex.resolve()
    if not rinex_path.is_file():
        raise SystemExit(f"找不到 RINEX 文件: {rinex_path}")
    with brdc._open_text(rinex_path) as fp:
        ephs = brdc.parse_rinex_nav_gps(fp)
    n_raw = len(ephs)
    if latest_per_prn:
        ephs = brdc.pick_latest_per_prn(ephs)
    n_pick = len(ephs)
    ephs = brdc.filter_by_max_age(ephs, max_age_hours)
    if not ephs:
        lines = [
            "RINEX 里没有可用的 GPS 星历（此步骤与串口/COM 无关，先检查下面几项）。",
            f"  文件: {rinex_path}",
            f"  解析到 GPS 星历 {n_raw} 条 → 按星取最新后 {n_pick} 条 → 时效筛选后 0 条。",
        ]
        if n_raw == 0:
            lines.append(
                "  原因可能是：不是 GPS NAV（v2 *.**n 或 v3 含 Gxx），或文件损坏/截断；请换当天 brdc 或 BRDC *MN.rnx。"
            )
        elif max_age_hours > 0:
            lines.append(
                f"  多为星历 TOC 相对本机 UTC 已超过 {max_age_hours:g} h；可加参数 --max-age-hours 0 或 168，"
                "并确认 Windows「日期和时间」正确（含时区）。"
            )
        raise SystemExit("\n".join(lines))
    return b"".join(brdc.encode_rtcm1019_frame(e) for e in ephs)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="NTRIP RTCM + RINEX→1019 合并输出到串口（一键补齐星历）"
    )
    ap.add_argument(
        "--rinex",
        type=Path,
        required=True,
        help="当天 GPS 广播 NAV：v2（如仓库根目录 brdc1250.26n）或 v3 混合 NAV（*MN.rnx），可选 .gz",
    )
    ap.add_argument(
        "--caster",
        default="114.111.30.20",
        help="NTRIP caster：域名或 IP（默认全国/QX 示例 caster）",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=8003,
        help="caster 端口：8001=ITRF2008，8002=WGS84，8003=CGCS2000（默认）",
    )
    ap.add_argument(
        "--mount",
        default="RTCM33_GGB",
        help="NTRIP 挂载点（默认 RTCM33_GGB；亦可 AUTO 等，须与 caster 一致）",
    )
    ap.add_argument("--user", "-u", default="qx4839", help="NTRIP 账号（默认 qx4839）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--password", "-p", help="密码（慎用：会留在 shell 历史）")
    g.add_argument(
        "--password-env",
        metavar="VAR",
        help="从环境变量读密码，例如 NTRIP_PASSWORD",
    )
    ap.add_argument("--com", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--eph-interval", type=float, default=10.0, help="每隔多少秒注入一轮 1019")
    ap.add_argument("--connect-timeout", type=float, default=15.0)
    ap.add_argument("--reload-rinex", action="store_true", help="每次注入前重新读 RINEX（便于热替换文件）")
    ap.add_argument("--latest-per-prn", action="store_true", default=True)
    ap.add_argument("--no-latest-per-prn", action="store_false", dest="latest_per_prn")
    ap.add_argument(
        "--max-age-hours",
        type=float,
        default=48.0,
        help="丢弃 TOC 早于此刻超过该小时数的星历；0=不限制（brdc 偏旧时可用）",
    )
    args = ap.parse_args(argv)

    if args.password_env:
        pw = __import__("os").environ.get(args.password_env, "")
        if not pw:
            raise SystemExit(f"环境变量 {args.password_env!r} 为空")
    else:
        pw = args.password or ""

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("请先安装: pip install pyserial") from exc

    stop = threading.Event()
    tx_q: "queue.Queue[bytes | None]" = queue.Queue()

    def handle_sig(*_a):
        stop.set()

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    try:
        eph_blob = load_1019_batch(args.rinex, args.latest_per_prn, args.max_age_hours)
    except SystemExit:
        raise
    except Exception as exc:
        raise SystemExit(str(exc)) from exc

    def open_serial():
        try:
            return serial.Serial(args.com, baudrate=args.baud, timeout=0.05)
        except (serial.SerialException, OSError, PermissionError) as exc:
            raise SystemExit(_serial_open_diagnostic(args.com, exc)) from exc

    ser_holder: list = [open_serial()]

    def reopen_serial_quiet() -> bool:
        """Return True if port reopened (for writer thread); never raises SystemExit."""
        try:
            try:
                ser_holder[0].close()
            except OSError:
                pass
            ser_holder[0] = serial.Serial(args.com, baudrate=args.baud, timeout=0.05)
            return True
        except (serial.SerialException, OSError, PermissionError) as exc:
            print(f"重新打开串口 {args.com!r} 失败: {exc}", flush=True)
            return False

    def serial_writer() -> None:
        """
        Windows/pyserial：多线程同时对同一 Serial 实例 write 易触发 WinError 5「拒绝访问」，
        与「是否有其它软件」无关。此处仅本线程调用 write。
        """
        while True:
            item = tx_q.get()
            if item is None:
                break
            for attempt in range(3):
                try:
                    ser_holder[0].write(item)
                    break
                except (serial.SerialException, PermissionError, OSError) as exc:
                    if attempt < 2 and reopen_serial_quiet():
                        time.sleep(0.05)
                        continue
                    print(
                        f"串口写入失败（已重试）: {exc} — 若反复出现：拔插 USB、换 COM、"
                        f"执行 resmon.exe 查看「关联的句柄」里是否有进程占用 {args.com}。",
                        flush=True,
                    )
                    break

    def enqueue_tx(data: bytes) -> None:
        if not data:
            return
        tx_q.put(data)

    writer_th = threading.Thread(target=serial_writer, daemon=False)
    writer_th.start()

    print(
        f"已加载 1019 批次 {len(eph_blob)} 字节；连接 NTRIP {args.caster}:{args.port}/{args.mount} …",
        flush=True,
    )

    # 启动前先灌一轮星历，便于 MCU 尽快有 eph（经队列，由 serial_writer 写出）
    enqueue_tx(eph_blob)

    def ntrip_loop() -> None:
        backoff = 1.0
        while not stop.is_set():
            try:
                sock = ntrip_open_stream(
                    args.caster,
                    args.port,
                    args.mount,
                    args.user,
                    pw,
                    args.connect_timeout,
                )
                backoff = 1.0
                print("NTRIP 已连接，转发差分数据…", flush=True)
                while not stop.is_set():
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    enqueue_tx(chunk)
                sock.close()
                print("NTRIP 断开，将重连…", flush=True)
            except OSError as exc:
                print(f"NTRIP 错误: {exc}；{backoff:.0f}s 后重试", flush=True)
            except Exception as exc:
                print(f"NTRIP 错误: {exc}；{backoff:.0f}s 后重试", flush=True)
            time.sleep(backoff)
            backoff = min(backoff * 2, 60.0)

    th = threading.Thread(target=ntrip_loop, daemon=True)
    th.start()

    next_inj = time.monotonic()
    try:
        while not stop.is_set():
            if time.monotonic() >= next_inj:
                try:
                    blob = (
                        load_1019_batch(args.rinex, args.latest_per_prn, args.max_age_hours)
                        if args.reload_rinex
                        else eph_blob
                    )
                    enqueue_tx(blob)
                except Exception as exc:
                    print(f"准备 1019 批次失败: {exc}", flush=True)
                next_inj = time.monotonic() + args.eph_interval
            time.sleep(0.05)
    finally:
        stop.set()
        tx_q.put(None)
        writer_th.join(timeout=3.0)
        try:
            ser_holder[0].close()
        except OSError:
            pass
        print("已退出。", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
