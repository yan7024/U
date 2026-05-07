#!/usr/bin/env python3
"""
官方 / RINEX 回放 → USART3（CH340）：RTCM 观测字节流 + 周期性 1019（广播星历）。

MCU 固件须 **编译加 -DMCU_REPLAY_RTCM_OBS=1**，USART3 收到的 MSM（如 1074/1077）
decode 后会复制到 rover->obs 做 SPP；USART1 可断开 F9P。

① 用 RTKLIB 桌面程序把 RINEX 观测转成 RTCM 二进制文件（示例，路径按你机器改）：

  str2str -in file://D:/data/site.obs#rinexo -out file://D:/data/replay.rtcm#rtcm3 -msg 1077

若需旧式 GPS：可把 -msg 改成 1004。导航星历仍请用下方 --rinex（如仓库根目录 brdc1250.26n）注 1019。

② 回放（合并 1019，单进程独占 COM）：

  python tools/replay_rtcm_to_mcu.py --rtcm D:/data/replay.rtcm --rinex D:/ST/UBLOX/U/brdc1250.26n --com COM7

可选 --realtime：按文件长度与 1Hz 观测粗算Sleep，避免灌太快 MCU 来不及解；默认尽速发送。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import threading
import time

_TOOLS = pathlib.Path(__file__).resolve().parent
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import brdc_to_rtcm1019 as brdc  # noqa: E402


def load_1019_blob(rinex: pathlib.Path, latest_per_prn: bool, max_age_hours: float) -> bytes:
    with brdc._open_text(rinex.resolve()) as fp:
        ephs = brdc.parse_rinex_nav_gps(fp)
    if latest_per_prn:
        ephs = brdc.pick_latest_per_prn(ephs)
    ephs = brdc.filter_by_max_age(ephs, max_age_hours)
    if not ephs:
        raise SystemExit("没有可用 GPS 星历（检查 brdc 日期与 --max-age-hours）")
    return b"".join(brdc.encode_rtcm1019_frame(e) for e in ephs)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="RTCM 文件回放 + brdc 1019 → 串口（MCU 回放模式）")
    ap.add_argument("--rtcm", type=pathlib.Path, required=True, help="str2str 生成的 .rtcm 二进制")
    ap.add_argument("--rinex", type=pathlib.Path, required=True, help="GPS 广播 NAV（如 brdc1250.26n），用于 1019")
    ap.add_argument("--com", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--eph-interval", type=float, default=15.0)
    ap.add_argument("--max-age-hours", type=float, default=0.0)
    ap.add_argument("--chunk", type=int, default=512, help="每次写入串口字节数")
    ap.add_argument(
        "--realtime",
        action="store_true",
        help="按约 1Hz 观测节奏节流发送（粗判：帧数/观测历元）",
    )
    args = ap.parse_args(argv)

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pip install pyserial") from exc

    rtcm_path = args.rtcm.resolve()
    if not rtcm_path.is_file():
        raise SystemExit(f"找不到 RTCM 文件: {rtcm_path}")

    eph_blob = load_1019_blob(args.rinex, True, args.max_age_hours)
    data = rtcm_path.read_bytes()
    if len(data) == 0:
        raise SystemExit("RTCM 文件为空")

    ser = serial.Serial(args.com, baudrate=args.baud, timeout=0.05)
    lock = threading.Lock()

    def inj_loop() -> None:
        next_t = time.monotonic()
        while True:
            try:
                with lock:
                    ser.write(eph_blob)
                    ser.flush()
            except OSError as exc:
                print(f"1019 写入失败: {exc}", flush=True)
            next_t += args.eph_interval
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    th = threading.Thread(target=inj_loop, daemon=True)
    th.start()

    # 粗算实时节奏：数 RTCM 帧（0xD3），假定观测约每帧/历元 ~1s（仅近似）
    frame_markers = data.count(bytes([0xD3]))
    if args.realtime and frame_markers > 0:
        sec_per_byte = max(frame_markers, 1) / float(len(data))
        pause = sec_per_byte * args.chunk
    else:
        pause = 0.0

    print(
        f"发送 {len(data)} 字节 RTCM + 每 {args.eph_interval:g}s 一轮 1019 ({len(eph_blob)} B) → {args.com}",
        flush=True,
    )
    try:
        off = 0
        while off < len(data):
            chunk = data[off : off + args.chunk]
            off += len(chunk)
            with lock:
                ser.write(chunk)
                ser.flush()
            if pause > 0:
                time.sleep(pause)
        print("RTCM 文件已发完；1019 线程仍在运行（Ctrl+C 退出）。", flush=True)
        while True:
            time.sleep(3600.0)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
