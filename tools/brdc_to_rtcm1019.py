#!/usr/bin/env python3
"""
Broadcast GPS RINEX navigation (e.g. daily BRDC *.rnx / *MN.rnx) -> RTCM 3.x type 1019
frames, aligned with RTKLIB encode_type1019 + gen_rtcm3 framing (CRC24Q).

Typical use (feed MCU USART3 / CH340 alongside MSM corrections):
  python tools/brdc_to_rtcm1019.py --rinex brdc1250.26n --com COM7 --baud 115200

Also accepts RINEX v3 mixed NAV (e.g. IGS BRDC *MN.rnx) or .gz:
  python tools/brdc_to_rtcm1019.py --rinex brdc1250.26n.gz --stdout --once

TCP client:
  python brdc_to_rtcm1019.py --rinex nav.rnx --tcp 192.168.1.10:2101 --interval 15

stdout pipe (e.g. into socat / another relay):
  python brdc_to_rtcm1019.py --rinex nav.rnx --stdout > raw.bin

Deps: Python 3.8+. Serial output needs: pip install pyserial
"""

from __future__ import annotations

import argparse
import gzip
import math
import re
import socket
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import BinaryIO, Iterable, List, Optional, TextIO

# --- RTKLIB-compatible constants (rinex.c / rtcm3e.c / rtkcmn.c) ---
P2_5 = 3.125e-2
P2_19 = 1.907348632812500e-6
P2_29 = 1.862645149230957e-9
P2_31 = 4.656612873077393e-10
P2_33 = 1.164153218269348e-10
P2_43 = 1.136868377216160e-13
P2_55 = 2.775557561562891e-17
SC2RAD = math.pi / 180.0

URA_EPH = (
    2.4,
    3.4,
    4.85,
    6.85,
    9.65,
    13.65,
    24.0,
    48.0,
    96.0,
    192.0,
    384.0,
    768.0,
    1536.0,
    3072.0,
    6144.0,
    0.0,
)

RTCM3PREAMB = 0xD3

TBL_CRC24Q = (
    0x000000,
    0x864CFB,
    0x8AD50D,
    0x0C99F6,
    0x93E6E1,
    0x15AA1A,
    0x1933EC,
    0x9F7F17,
    0xA18139,
    0x27CDC2,
    0x2B5434,
    0xAD18CF,
    0x3267D8,
    0xB42B23,
    0xB8B2D5,
    0x3EFE2E,
    0xC54E89,
    0x430272,
    0x4F9B84,
    0xC9D77F,
    0x56A868,
    0xD0E493,
    0xDC7D65,
    0x5A319E,
    0x64CFB0,
    0xE2834B,
    0xEE1ABD,
    0x685646,
    0xF72951,
    0x7165AA,
    0x7DFC5C,
    0xFBB0A7,
    0x0CD1E9,
    0x8A9D12,
    0x8604E4,
    0x00481F,
    0x9F3708,
    0x197BF3,
    0x15E205,
    0x93AEFE,
    0xAD50D0,
    0x2B1C2B,
    0x2785DD,
    0xA1C926,
    0x3EB631,
    0xB8FACA,
    0xB4633C,
    0x322FC7,
    0xC99F60,
    0x4FD39B,
    0x434A6D,
    0xC50696,
    0x5A7981,
    0xDC357A,
    0xD0AC8C,
    0x56E077,
    0x681E59,
    0xEE52A2,
    0xE2CB54,
    0x6487AF,
    0xFBF8B8,
    0x7DB443,
    0x712DB5,
    0xF7614E,
    0x19A3D2,
    0x9FEF29,
    0x9376DF,
    0x153A24,
    0x8A4533,
    0x0C09C8,
    0x00903E,
    0x86DCC5,
    0xB822EB,
    0x3E6E10,
    0x32F7E6,
    0xB4BB1D,
    0x2BC40A,
    0xAD88F1,
    0xA11107,
    0x275DFC,
    0xDCED5B,
    0x5AA1A0,
    0x563856,
    0xD074AD,
    0x4F0BBA,
    0xC94741,
    0xC5DEB7,
    0x43924C,
    0x7D6C62,
    0xFB2099,
    0xF7B96F,
    0x71F594,
    0xEE8A83,
    0x68C678,
    0x645F8E,
    0xE21375,
    0x15723B,
    0x933EC0,
    0x9FA736,
    0x19EBCD,
    0x8694DA,
    0x00D821,
    0x0C41D7,
    0x8A0D2C,
    0xB4F302,
    0x32BFF9,
    0x3E260F,
    0xB86AF4,
    0x2715E3,
    0xA15918,
    0xADC0EE,
    0x2B8C15,
    0xD03CB2,
    0x567049,
    0x5AE9BF,
    0xDCA544,
    0x43DA53,
    0xC596A8,
    0xC90F5E,
    0x4F43A5,
    0x71BD8B,
    0xF7F170,
    0xFB6886,
    0x7D247D,
    0xE25B6A,
    0x641791,
    0x688E67,
    0xEEC29C,
    0x3347A4,
    0xB50B5F,
    0xB992A9,
    0x3FDE52,
    0xA0A145,
    0x26EDBE,
    0x2A7448,
    0xAC38B3,
    0x92C69D,
    0x148A66,
    0x181390,
    0x9E5F6B,
    0x01207C,
    0x876C87,
    0x8BF571,
    0x0DB98A,
    0xF6092D,
    0x7045D6,
    0x7CDC20,
    0xFA90DB,
    0x65EFCC,
    0xE3A337,
    0xEF3AC1,
    0x69763A,
    0x578814,
    0xD1C4EF,
    0xDD5D19,
    0x5B11E2,
    0xC46EF5,
    0x42220E,
    0x4EBBF8,
    0xC8F703,
    0x3F964D,
    0xB9DAB6,
    0xB54340,
    0x330FBB,
    0xAC70AC,
    0x2A3C57,
    0x26A5A1,
    0xA0E95A,
    0x9E1774,
    0x185B8F,
    0x14C279,
    0x928E82,
    0x0DF195,
    0x8BBD6E,
    0x872498,
    0x016863,
    0xFAD8C4,
    0x7C943F,
    0x700DC9,
    0xF64132,
    0x693E25,
    0xEF72DE,
    0xE3EB28,
    0x65A7D3,
    0x5B59FD,
    0xDD1506,
    0xD18CF0,
    0x57C00B,
    0xC8BF1C,
    0x4EF3E7,
    0x426A11,
    0xC426EA,
    0x2AE476,
    0xACA88D,
    0xA0317B,
    0x267D80,
    0xB90297,
    0x3F4E6C,
    0x33D79A,
    0xB59B61,
    0x8B654F,
    0x0D29B4,
    0x01B042,
    0x87FCB9,
    0x1883AE,
    0x9ECF55,
    0x9256A3,
    0x141A58,
    0xEFAAFF,
    0x69E604,
    0x657FF2,
    0xE33309,
    0x7C4C1E,
    0xFA00E5,
    0xF69913,
    0x70D5E8,
    0x4E2BC6,
    0xC8673D,
    0xC4FECB,
    0x42B230,
    0xDDCD27,
    0x5B81DC,
    0x57182A,
    0xD154D1,
    0x26359F,
    0xA07964,
    0xACE092,
    0x2AAC69,
    0xB5D37E,
    0x339F85,
    0x3F0673,
    0xB94A88,
    0x87B4A6,
    0x01F85D,
    0x0D61AB,
    0x8B2D50,
    0x145247,
    0x921EBC,
    0x9E874A,
    0x18CBB1,
    0xE37B16,
    0x6537ED,
    0x69AE1B,
    0xEFE2E0,
    0x709DF7,
    0xF6D10C,
    0xFA48FA,
    0x7C0401,
    0x42FA2F,
    0xC4B6D4,
    0xC82F22,
    0x4E63D9,
    0xD11CCE,
    0x575035,
    0x5BC9C3,
    0xDD8538,
)


def rtk_crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ TBL_CRC24Q[(crc >> 16) ^ b]
    return crc


def setbitu(buff: bytearray, pos: int, nbits: int, val: int) -> None:
    val &= (1 << nbits) - 1
    for k in range(nbits):
        bit = (val >> (nbits - 1 - k)) & 1
        byte_idx = (pos + k) // 8
        bit_idx = 7 - ((pos + k) % 8)
        if bit:
            buff[byte_idx] |= 1 << bit_idx
        else:
            buff[byte_idx] &= ~(1 << bit_idx)


def setbits(buff: bytearray, pos: int, nbits: int, val: int) -> None:
    mask = (1 << nbits) - 1
    setbitu(buff, pos, nbits, val & mask)


def _round_i(x: float) -> int:
    return int(math.floor(x + 0.5) if x >= 0.0 else -math.floor(-x + 0.5))


def _round_u(x: float) -> int:
    return int(math.floor(x + 0.5)) & 0xFFFFFFFF


def uraindex(value: float) -> int:
    for i in range(15):
        if URA_EPH[i] >= value:
            return i
    return 15


def ura_from_rinex_field(value: float) -> int:
    """RINEX stores either URA index (0..15) or a nominal length in metres."""
    if value == int(value) and 0 <= value <= 15:
        return int(value)
    return uraindex(value)


def julian_day(y: int, m: int, d: int, h: int, mi: int, s: float) -> float:
    if m <= 2:
        y -= 1
        m += 12
    a = y // 100
    b = 2 - a + a // 4
    jd = int(365.25 * (y + 4716)) + int(30.6001 * (m + 1)) + d + b - 1524.5
    jd += (h + mi / 60.0 + s / 3600.0) / 24.0
    return jd


def calendar_to_gpst_wsow(y: int, m: int, d: int, h: int, mi: int, s: float) -> tuple[int, float]:
    """GPS week and seconds-of-week (continuous SI seconds), ICD-style."""
    t = (julian_day(y, m, d, h, mi, s) - 2444244.5) * 86400.0
    week = int(t // 604800)
    sow = t - week * 604800
    return week, sow


def gpst_seconds_from_datetime(dt: datetime) -> float:
    """Continuous GPST seconds since GPS epoch (for comparing TOE vs now)."""
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)
    sec = dt.second + dt.microsecond / 1e6
    w, sow = calendar_to_gpst_wsow(dt.year, dt.month, dt.day, dt.hour, dt.minute, sec)
    return w * 604800.0 + sow


def toe_gpst_seconds(e: GpsEph) -> float:
    return float(e.week) * 604800.0 + float(e.toes)


@dataclass
class GpsEph:
    prn: int
    toc_dt: datetime  # reference time for clock (RINEX epoch)
    week: int  # GPS week from navigation record (broadcast)
    toes: float  # time of ephemeris [s] in week
    sqrt_a: float  # sqrt(A)
    e: float
    i0: float
    omg0: float
    omg: float
    m0: float
    deln: float
    omgd: float
    idot: float
    crc: float
    crs: float
    cuc: float
    cus: float
    cic: float
    cis: float
    f0: float
    f1: float
    f2: float
    iode: int
    iodc: int
    svh: int
    sva: int
    code: int
    flag: int
    tgd0: float
    fit_hours: float


def _open_text(path: Path):
    if path.suffix.lower() == ".gz" or str(path).lower().endswith(".gz"):
        return gzip.open(path, "rt", encoding="ascii", errors="replace")
    return path.open("r", encoding="ascii", errors="replace")


def _rinex_version(header_lines: List[str]) -> float:
    for ln in header_lines:
        if "RINEX VERSION / TYPE" in ln:
            return float(ln[:20].strip())
    return 0.0


def _read_rinex_header(fp: TextIO) -> List[str]:
    header: List[str] = []
    while True:
        line = fp.readline()
        if not line:
            break
        header.append(line.rstrip("\n"))
        if "END OF HEADER" in line:
            break
    return header


def _parse_rinex2_epoch(seg: str) -> datetime:
    """19-char RINEX2 epoch field: yy mm dd hh mm ss.sss (RTKLIB str2time)."""
    parts = seg.strip().split()
    if len(parts) < 6:
        raise ValueError(f"bad RINEX2 epoch: {seg!r}")
    yy, mo, d, h, mi = map(int, parts[:5])
    sec = float(parts[5])
    year = yy + (2000 if yy < 80 else 1900)
    sec_i = int(sec)
    micro = int(round((sec - sec_i) * 1e6))
    return datetime(year, mo, d, h, mi, sec_i, micro, tzinfo=timezone.utc)


def _build_gps_eph_from_nav_data(prn: int, toc: datetime, data: List[float]) -> GpsEph:
    sqrt_a = data[10]
    return GpsEph(
        prn=prn,
        toc_dt=toc,
        week=int(data[21]),
        toes=float(data[11]),
        sqrt_a=sqrt_a,
        e=float(data[8]),
        i0=float(data[15]),
        omg0=float(data[13]),
        omg=float(data[17]),
        m0=float(data[6]),
        deln=float(data[5]),
        omgd=float(data[18]),
        idot=float(data[19]),
        crc=float(data[16]),
        crs=float(data[4]),
        cuc=float(data[7]),
        cus=float(data[9]),
        cic=float(data[12]),
        cis=float(data[14]),
        f0=float(data[0]),
        f1=float(data[1]),
        f2=float(data[2]),
        iode=int(data[3]),
        iodc=int(data[26]),
        svh=int(data[24]),
        sva=ura_from_rinex_field(float(data[23])),
        code=int(data[20]),
        flag=int(data[22]),
        tgd0=float(data[25]),
        fit_hours=float(data[28]),
    )


def parse_rinex2_nav_gps(fp: TextIO) -> List[GpsEph]:
    """GPS NAV body for RINEX 2.x (e.g. brdc*.**n). fp must start after END OF HEADER."""
    out: List[GpsEph] = []
    data = [0.0] * 64
    i = 0
    prn = 0
    toc = datetime(1980, 1, 6, tzinfo=timezone.utc)

    while True:
        line = fp.readline()
        if not line:
            break
        line = line.rstrip("\n\r")
        if not line.strip():
            continue

        if i == 0:
            prn_s = line[0:2].strip()
            if not prn_s.isdigit():
                continue
            prn = int(prn_s)
            if not (1 <= prn <= 32):
                continue
            if len(line) < 22:
                continue
            try:
                toc = _parse_rinex2_epoch(line[3:22])
            except ValueError:
                continue
            data[0] = _str2num(line, 22, 19)
            data[1] = _str2num(line, 41, 19)
            data[2] = _str2num(line, 60, 19)
            i = 3
        else:
            for j in range(4):
                data[i] = _str2num(line, 3 + j * 19, 19)
                i += 1
            if i >= 31:
                out.append(_build_gps_eph_from_nav_data(prn, toc, data))
                i = 0

    return out


def _parse_epoch_tokens(line: str, sp: int) -> datetime:
    epoch = line[sp : sp + 19]
    parts = epoch.split()
    if len(parts) < 6:
        raise ValueError(f"bad epoch: {epoch!r}")
    y, mo, d, h, mi = map(int, parts[:5])
    sec = float(parts[5])
    sec_i = int(sec)
    micro = int(round((sec - sec_i) * 1e6))
    return datetime(y, mo, d, h, mi, sec_i, micro, tzinfo=timezone.utc)


def _str2num(line: str, start: int, width: int) -> float:
    s = line[start : start + width].strip()
    if not s:
        return 0.0
    try:
        return float(s)
    except ValueError:
        return 0.0


def _parse_rinex3_nav_gps_body(fp: TextIO) -> List[GpsEph]:
    out: List[GpsEph] = []
    data_i = 0
    data = [0.0] * 64
    line0_sp = 4
    prn = 0
    toc = datetime(1980, 1, 6, tzinfo=timezone.utc)

    while True:
        line = fp.readline()
        if not line:
            break
        line = line.rstrip("\n")
        if not line.strip():
            continue

        if data_i == 0:
            if not line.startswith("G") or line[1] not in "0123456789":
                continue
            prn_s = line[:3].strip()[1:]
            prn = int(prn_s)
            toc = _parse_epoch_tokens(line, line0_sp)
            data[0] = _str2num(line, line0_sp + 19, 19)
            data[1] = _str2num(line, line0_sp + 38, 19)
            data[2] = _str2num(line, line0_sp + 57, 19)
            data_i = 3
            continue

        for j in range(4):
            data[data_i] = _str2num(line, line0_sp + j * 19, 19)
            data_i += 1

        if data_i >= 31:
            out.append(_build_gps_eph_from_nav_data(prn, toc, data))
            data_i = 0

    return out


def parse_rinex_nav_gps(fp: TextIO) -> List[GpsEph]:
    """Read RINEX NAV header from fp, then GPS ephemerides (v2 *.**n or v3+ Gxx)."""
    header = _read_rinex_header(fp)
    if not header or "END OF HEADER" not in header[-1]:
        raise RuntimeError("RINEX: missing END OF HEADER")
    ver = _rinex_version(header)
    if ver >= 3.0:
        return _parse_rinex3_nav_gps_body(fp)
    if ver >= 2.0:
        return parse_rinex2_nav_gps(fp)
    raise RuntimeError(f"RINEX version {ver} not supported (need v2 GPS NAV or v3+ Gxx).")


def parse_rinex3_nav_gps(fp: TextIO) -> List[GpsEph]:
    """Backward-compatible: full file parse; v2/v3 handled via header version."""
    return parse_rinex_nav_gps(fp)


def encode_rtcm1019_frame(eph: GpsEph) -> bytes:
    """One RTCM 3 frame: type 1019 GPS ephemeris."""
    a = eph.sqrt_a * eph.sqrt_a
    week10 = eph.week % 1024
    toe_u16 = _round_u(eph.toes / 16.0) & 0xFFFF
    y, mo, d, h, mi = (
        eph.toc_dt.year,
        eph.toc_dt.month,
        eph.toc_dt.day,
        eph.toc_dt.hour,
        eph.toc_dt.minute,
    )
    sec = eph.toc_dt.second + eph.toc_dt.microsecond / 1e6
    _, toc_sow = calendar_to_gpst_wsow(y, mo, d, h, mi, sec)
    toc_u16 = _round_u(toc_sow / 16.0) & 0xFFFF

    sqrt_a_u = _round_u(math.sqrt(a) / P2_19)
    e_u = _round_u(eph.e / P2_33)
    i0_i = _round_i(eph.i0 / P2_31 / SC2RAD)
    omg0_i = _round_i(eph.omg0 / P2_31 / SC2RAD)
    omg_i = _round_i(eph.omg / P2_31 / SC2RAD)
    m0_i = _round_i(eph.m0 / P2_31 / SC2RAD)
    deln_i = _round_i(eph.deln / P2_43 / SC2RAD)
    idot_i = _round_i(eph.idot / P2_43 / SC2RAD)
    omgd_i = _round_i(eph.omgd / P2_43 / SC2RAD)
    crs_i = _round_i(eph.crs / P2_5)
    crc_i = _round_i(eph.crc / P2_5)
    cus_i = _round_i(eph.cus / P2_29)
    cuc_i = _round_i(eph.cuc / P2_29)
    cis_i = _round_i(eph.cis / P2_29)
    cic_i = _round_i(eph.cic / P2_29)
    af0_i = _round_i(eph.f0 / P2_31)
    af1_i = _round_i(eph.f1 / P2_43)
    af2_i = _round_i(eph.f2 / P2_55)
    tgd_i = _round_i(eph.tgd0 / P2_31)

    nbits = 512
    buff = bytearray((nbits + 7) // 8)
    i = 0
    setbitu(buff, i, 8, RTCM3PREAMB)
    i += 8
    setbitu(buff, i, 6, 0)
    i += 6
    setbitu(buff, i, 10, 0)
    i += 10

    setbitu(buff, i, 12, 1019)
    i += 12
    setbitu(buff, i, 6, eph.prn)
    i += 6
    setbitu(buff, i, 10, week10)
    i += 10
    setbitu(buff, i, 4, eph.sva & 0xF)
    i += 4
    setbitu(buff, i, 2, eph.code & 0x3)
    i += 2
    setbits(buff, i, 14, idot_i)
    i += 14
    setbitu(buff, i, 8, eph.iode & 0xFF)
    i += 8
    setbitu(buff, i, 16, toc_u16)
    i += 16
    setbits(buff, i, 8, af2_i)
    i += 8
    setbits(buff, i, 16, af1_i)
    i += 16
    setbits(buff, i, 22, af0_i)
    i += 22
    setbitu(buff, i, 10, eph.iodc & 0x3FF)
    i += 10
    setbits(buff, i, 16, crs_i)
    i += 16
    setbits(buff, i, 16, deln_i)
    i += 16
    setbits(buff, i, 32, m0_i)
    i += 32
    setbits(buff, i, 16, cuc_i)
    i += 16
    setbitu(buff, i, 32, e_u)
    i += 32
    setbits(buff, i, 16, cus_i)
    i += 16
    setbitu(buff, i, 32, sqrt_a_u)
    i += 32
    setbitu(buff, i, 16, toe_u16)
    i += 16
    setbits(buff, i, 16, cic_i)
    i += 16
    setbits(buff, i, 32, omg0_i)
    i += 32
    setbits(buff, i, 16, cis_i)
    i += 16
    setbits(buff, i, 32, i0_i)
    i += 32
    setbits(buff, i, 16, crc_i)
    i += 16
    setbits(buff, i, 32, omg_i)
    i += 32
    setbits(buff, i, 24, omgd_i)
    i += 24
    setbits(buff, i, 8, tgd_i)
    i += 8
    setbitu(buff, i, 6, eph.svh & 0x3F)
    i += 6
    setbitu(buff, i, 1, eph.flag & 1)
    i += 1
    setbitu(buff, i, 1, 1 if eph.fit_hours > 4.0 else 0)
    i += 1

    while i % 8:
        setbitu(buff, i, 1, 0)
        i += 1
    msg_len = i // 8
    setbitu(buff, 14, 10, msg_len - 3)
    crc = rtk_crc24q(buff[:msg_len])
    return buff[:msg_len] + struct.pack(">I", crc)[1:4]


def pick_latest_per_prn(ephs: Iterable[GpsEph]) -> List[GpsEph]:
    """
    One broadcast ephemeris per PRN. Prefer TOE nearest current GPST — not latest TOC.
    BRDC has many blocks per satellite; latest TOC can leave TOE hours away from 'now',
    so seleph() on the MCU rejects (ephA>0, ephG=0, lack of valid sats).
    """
    t_ref = gpst_seconds_from_datetime(datetime.now(timezone.utc))
    best: dict[int, tuple[float, GpsEph]] = {}
    for e in ephs:
        d = abs(toe_gpst_seconds(e) - t_ref)
        cur = best.get(e.prn)
        if cur is None or d < cur[0]:
            best[e.prn] = (d, e)
    return sorted((x[1] for x in best.values()), key=lambda x: x.prn)


def filter_by_max_age(ephs: Iterable[GpsEph], max_age_hours: float) -> List[GpsEph]:
    if max_age_hours <= 0:
        return list(ephs)
    now = datetime.now(timezone.utc)
    lim = max_age_hours * 3600.0
    out = []
    for e in ephs:
        if (now - e.toc_dt).total_seconds() <= lim:
            out.append(e)
    return out


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Install pyserial: pip install pyserial") from exc
    return serial.Serial(port, baudrate=baud, timeout=0)


def parse_tcp_spec(spec: str) -> tuple[str, int]:
    host, _, port_s = spec.partition(":")
    if not port_s:
        raise ValueError("tcp needs host:port")
    return host.strip(), int(port_s.strip())


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="RINEX GPS NAV -> RTCM3 1019 stream")
    ap.add_argument(
        "--rinex",
        required=True,
        type=Path,
        help="GPS NAV: RINEX v2 (*.**n, e.g. brdc1250.26n) or v3 mixed NAV with Gxx (*.rnx), optional .gz",
    )
    ap.add_argument("--latest-per-prn", action="store_true", default=True)
    ap.add_argument("--no-latest-per-prn", action="store_false", dest="latest_per_prn")
    ap.add_argument("--max-age-hours", type=float, default=48.0, help="drop older TOC; 0 = keep all")
    ap.add_argument("--interval", type=float, default=10.0, help="repeat full batch every N seconds")
    ap.add_argument("--once", action="store_true", help="send one batch and exit")
    ap.add_argument("--stdout", action="store_true", help="write binary RTCM to stdout")
    ap.add_argument("--com", help="serial port (Windows COMx)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp", metavar="HOST:PORT", help="TCP client connect target")
    args = ap.parse_args(argv)

    sinks: List[BinaryIO] = []
    ser = None
    sock = None
    try:
        if args.stdout:
            sinks.append(sys.stdout.buffer)
        if args.com:
            ser = open_serial(args.com, args.baud)
            sinks.append(ser)
        if args.tcp:
            host, port = parse_tcp_spec(args.tcp)
            sock = socket.create_connection((host, port), timeout=10.0)
            sinks.append(sock.makefile("wb"))
        if not sinks:
            raise SystemExit("Pick at least one of --stdout --com --tcp")

        with _open_text(args.rinex) as fp:
            ephs = parse_rinex_nav_gps(fp)

        if args.latest_per_prn:
            ephs = pick_latest_per_prn(ephs)
        ephs = filter_by_max_age(ephs, args.max_age_hours)
        if not ephs:
            raise SystemExit("No GPS ephemerides after filters.")

        frames = [encode_rtcm1019_frame(e) for e in ephs]
        batch_bytes = sum(len(f) for f in frames)

        def send_batch() -> None:
            payload = b"".join(frames)
            for snk in sinks:
                snk.write(payload)
                snk.flush()

        if args.com:
            print(
                f"Serial {args.com} @ {args.baud} (8N1). "
                f"RINEX {args.rinex} → {len(frames)}×RTCM1019, {batch_bytes} B/batch.",
                flush=True,
            )
        if args.stdout:
            print("Also writing binary RTCM to stdout.", flush=True)
        if args.once:
            print("Mode: --once (single batch, then exit).", flush=True)
        else:
            print(
                f"Mode: repeat every {args.interval:g} s — Ctrl+C to stop. "
                f"若终端无新行是正常现象，数据在持续写出。",
                flush=True,
            )

        send_batch()
        print(f"  sent batch #1 ({batch_bytes} B)", flush=True)
        if not args.once:
            n = 1
            while True:
                time.sleep(args.interval)
                send_batch()
                n += 1
                print(f"  sent batch #{n} ({batch_bytes} B)", flush=True)
        else:
            print("Done.", flush=True)
    finally:
        if sock:
            sock.close()
        if ser:
            ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
