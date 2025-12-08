#!/usr/bin/env python3
"""beta99_pack: Build beta99 binary bundles from desidulate SSF/log CSVs.

Usage:
  python tools/beta99_pack.py --ssf tune.ssf.zst --log tune.log.zst --out tune.b99

Optionally pass --json for a debug dump of the parsed structure.
"""

from __future__ import annotations

import argparse
import io
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd
import zstandard as zstd

BETA99_MAGIC = b"B99F"
BETA99_VERSION = 1

DEFAULT_MAX_OPS_PER_SSF = 512

OP_SET_FREQ = 0x01
OP_SET_PW = 0x02
OP_SET_CTRL = 0x03
OP_SET_AD = 0x04
OP_SET_SR = 0x05
OP_SET_MOD_FREQ = 0x06
OP_SET_MOD_TEST = 0x07
OP_SET_FILTER_ROUTE = 0x08
OP_SET_FILTER_EXT = 0x09
OP_SET_FILTER_CUTOFF = 0x0A
OP_SET_FILTER_RES = 0x0B
OP_SET_FILTER_MODE = 0x0C
OP_SET_VOLUME = 0x0D

CONTROL_BITS = (
    ("gate1", 0),
    ("sync1", 1),
    ("ring1", 2),
    ("test1", 3),
    ("tri1", 4),
    ("saw1", 5),
    ("pulse1", 6),
    ("noise1", 7),
)
FILTER_MODE_BITS = (
    ("fltlo", 0),
    ("fltband", 1),
    ("flthi", 2),
)


@dataclass
class Beta99Op:
    delta: int
    opcode: int
    data: bytes


@dataclass
class Beta99SSF:
    hashid: int
    duration: int
    ops: List[Beta99Op]


@dataclass
class Beta99Trigger:
    delta: int
    ssf_index: int
    voice: int


def read_zstd_csv(path: Path) -> pd.DataFrame:
    dctx = zstd.ZstdDecompressor()
    with path.open("rb") as fh, dctx.stream_reader(fh) as reader:
        payload = reader.read()
    return pd.read_csv(io.BytesIO(payload))


def to_int(value: object) -> Optional[int]:
    if pd.isna(value):
        return None
    return int(value)


def pack_u16(value: int) -> bytes:
    return struct.pack("<H", value & 0xFFFF)


def build_control_value(state: Dict[str, int]) -> int:
    value = 0
    for name, bit in CONTROL_BITS:
        if state.get(name, 0):
            value |= (1 << bit)
    return value


def build_filter_mode_value(state: Dict[str, int]) -> int:
    mask = 0
    for name, bit in FILTER_MODE_BITS:
        if state.get(name, 0):
            mask |= (1 << bit)
    return mask


class Beta99Builder:
    def __init__(self, ssf_path: Path, log_path: Path, max_ops_per_ssf: int) -> None:
        self.ssf_df = read_zstd_csv(ssf_path)
        self.log_df = read_zstd_csv(log_path)
        self.ssfs: List[Beta99SSF] = []
        self.triggers: List[Beta99Trigger] = []
        self.hash_chunks: Dict[int, List[int]] = {}
        self.max_ops_per_ssf = max(1, max_ops_per_ssf)
        self._build_ssfs()
        self._build_triggers()

    def _build_ssfs(self) -> None:
        for hashid, group in self.ssf_df.groupby('hashid', sort=False):
            ops, duration = self._build_ops_for_group(group)
            hashid_int = int(hashid)
            chunk_indices: List[int] = []
            for chunk_ops in self._chunk_ops(ops):
                if not chunk_ops:
                    continue
                chunk_duration = self._ops_duration(chunk_ops)
                idx = len(self.ssfs)
                self.ssfs.append(Beta99SSF(hashid=hashid_int,
                                           duration=chunk_duration,
                                           ops=chunk_ops))
                chunk_indices.append(idx)
            if not chunk_indices:
                # still produce an empty entry so triggers can reference something
                idx = len(self.ssfs)
                self.ssfs.append(Beta99SSF(hashid=hashid_int, duration=duration, ops=[]))
                chunk_indices.append(idx)
            self.hash_chunks[hashid_int] = chunk_indices

    def _build_ops_for_group(self, group: pd.DataFrame) -> Tuple[List[Beta99Op], int]:
        ops: List[Beta99Op] = []
        prev_clock = 0
        duration = 0
        control_state = {name: 0 for name, _ in CONTROL_BITS}
        filter_mode_state = {name: 0 for name, _ in FILTER_MODE_BITS}
        voice_filter_route = None
        filter_ext = None
        last_ctrl = None
        last_ad = None
        last_sr = None
        last_freq = None
        last_pw = None
        last_mod_freq = None
        last_mod_test = None
        last_filter_cutoff = None
        last_filter_res = None
        last_filter_mode = None
        last_volume = None

        adsr_state = {"atk1": 0, "dec1": 0, "sus1": 0, "rel1": 0}

        for row in group.sort_values('clock', kind='mergesort').itertuples(index=False):
            clock = int(row.clock)
            delta = clock - prev_clock
            prev_clock = clock
            duration = clock
            row_ops: List[Beta99Op] = []
            pending_delta = delta

            def emit(opcode: int, payload: bytes) -> None:
                nonlocal pending_delta
                row_ops.append(Beta99Op(delta=pending_delta, opcode=opcode, data=payload))
                pending_delta = 0

            freq = to_int(row.freq1)
            if freq is not None and freq != last_freq:
                emit(OP_SET_FREQ, pack_u16(freq))
                last_freq = freq

            pw = to_int(row.pwduty1)
            if pw is not None and pw != last_pw:
                emit(OP_SET_PW, pack_u16(pw))
                last_pw = pw

            ctrl_changed = False
            for name, _ in CONTROL_BITS:
                val = to_int(getattr(row, name))
                if val is None:
                    continue
                control_state[name] = 1 if val else 0
                ctrl_changed = True
            if ctrl_changed:
                ctrl_value = build_control_value(control_state)
                if ctrl_value != last_ctrl:
                    emit(OP_SET_CTRL, bytes([ctrl_value]))
                    last_ctrl = ctrl_value

            atk = to_int(row.atk1)
            dec = to_int(row.dec1)
            if atk is not None:
                adsr_state['atk1'] = atk & 0x0F
            if dec is not None:
                adsr_state['dec1'] = dec & 0x0F
            if atk is not None or dec is not None:
                ad_val = ((adsr_state['atk1'] & 0x0F) << 4) | (adsr_state['dec1'] & 0x0F)
                if ad_val != last_ad:
                    emit(OP_SET_AD, bytes([ad_val]))
                    last_ad = ad_val

            sus = to_int(row.sus1)
            rel = to_int(row.rel1)
            if sus is not None:
                adsr_state['sus1'] = sus & 0x0F
            if rel is not None:
                adsr_state['rel1'] = rel & 0x0F
            if sus is not None or rel is not None:
                sr_val = ((adsr_state['sus1'] & 0x0F) << 4) | (adsr_state['rel1'] & 0x0F)
                if sr_val != last_sr:
                    emit(OP_SET_SR, bytes([sr_val]))
                    last_sr = sr_val

            mod_freq = to_int(row.freq3)
            if mod_freq is not None and mod_freq != last_mod_freq:
                emit(OP_SET_MOD_FREQ, pack_u16(mod_freq))
                last_mod_freq = mod_freq

            mod_test = to_int(row.test3)
            if mod_test is not None:
                mod_test_val = 1 if mod_test else 0
                if mod_test_val != last_mod_test:
                    emit(OP_SET_MOD_TEST, bytes([mod_test_val]))
                    last_mod_test = mod_test_val

            flt_route = to_int(row.flt1)
            if flt_route is not None:
                route_val = 1 if flt_route else 0
                if route_val != voice_filter_route:
                    emit(OP_SET_FILTER_ROUTE, bytes([route_val]))
                    voice_filter_route = route_val

            flt_ext = to_int(row.fltext)
            if flt_ext is not None:
                ext_val = 1 if flt_ext else 0
                if ext_val != filter_ext:
                    emit(OP_SET_FILTER_EXT, bytes([ext_val]))
                    filter_ext = ext_val

            cutoff = to_int(row.fltcoff)
            if cutoff is not None and cutoff != last_filter_cutoff:
                emit(OP_SET_FILTER_CUTOFF, pack_u16(cutoff))
                last_filter_cutoff = cutoff

            fltres = to_int(row.fltres)
            if fltres is not None and fltres != last_filter_res:
                emit(OP_SET_FILTER_RES, bytes([fltres & 0x0F]))
                last_filter_res = fltres & 0x0F

            mode_changed = False
            for name, _ in FILTER_MODE_BITS:
                val = to_int(getattr(row, name))
                if val is None:
                    continue
                filter_mode_state[name] = 1 if val else 0
                mode_changed = True
            if mode_changed:
                mode_val = build_filter_mode_value(filter_mode_state)
                if mode_val != last_filter_mode:
                    emit(OP_SET_FILTER_MODE, bytes([mode_val & 0x07]))
                    last_filter_mode = mode_val

            vol = to_int(row.vol)
            if vol is not None and vol != last_volume:
                emit(OP_SET_VOLUME, bytes([vol & 0x0F]))
                last_volume = vol & 0x0F

            ops.extend(row_ops)

        return ops, duration

    def _chunk_ops(self, ops: List[Beta99Op]) -> List[List[Beta99Op]]:
        if len(ops) <= self.max_ops_per_ssf:
            return [ops]
        chunks: List[List[Beta99Op]] = []
        for start in range(0, len(ops), self.max_ops_per_ssf):
            chunk = ops[start:start + self.max_ops_per_ssf]
            chunks.append(chunk)
        return chunks

    @staticmethod
    def _ops_duration(chunk_ops: List[Beta99Op]) -> int:
        total = 0
        for op in chunk_ops:
            total += op.delta
        return total

    def _build_triggers(self) -> None:
        prev_clock = 0
        triggers: List[Beta99Trigger] = []
        for row in self.log_df.sort_values('clock', kind='mergesort').itertuples(index=False):
            clock = int(row.clock)
            delta = clock - prev_clock
            prev_clock = clock
            hashid = int(row.hashid)
            voice = int(row.voice)
            if hashid not in self.hash_chunks:
                raise ValueError(f"Trigger references unknown SSF hash {hashid}")
            chunk_indices = self.hash_chunks[hashid]
            for i, chunk_idx in enumerate(chunk_indices):
                trig_delta = delta if i == 0 else 0
                triggers.append(Beta99Trigger(delta=trig_delta, ssf_index=chunk_idx, voice=voice))
        self.triggers = triggers


def write_beta99(builder: Beta99Builder, out_path: Path) -> None:
    if len(builder.ssfs) > 0xFFFF:
        raise ValueError("Too many SSFs for 16-bit indices")
    with out_path.open('wb') as fh:
        fh.write(struct.pack('<4sHHII', BETA99_MAGIC, BETA99_VERSION, 0,
                             len(builder.ssfs), len(builder.triggers)))
        for ssf in builder.ssfs:
            fh.write(struct.pack('<QII', ssf.hashid & 0xFFFFFFFFFFFFFFFF,
                                 ssf.duration & 0xFFFFFFFF,
                                 len(ssf.ops)))
            for op in ssf.ops:
                data = op.data
                if len(data) > 4:
                    raise ValueError(f"Operation payload too large ({len(data)} bytes)")
                opcode = op.opcode & 0xFF
                length = len(data)
                if opcode in (OP_SET_CTRL, OP_SET_AD, OP_SET_SR, OP_SET_MOD_TEST,
                              OP_SET_FILTER_ROUTE, OP_SET_FILTER_EXT,
                              OP_SET_FILTER_RES, OP_SET_FILTER_MODE,
                              OP_SET_VOLUME):
                    if length != 1:
                        raise ValueError(f"Opcode {opcode:#x} expects 1 byte, got {length}")
                elif opcode in (OP_SET_FREQ, OP_SET_PW,
                                OP_SET_MOD_FREQ, OP_SET_FILTER_CUTOFF):
                    if length != 2:
                        raise ValueError(f"Opcode {opcode:#x} expects 2 bytes, got {length}")
                else:
                    if length > 4:
                        raise ValueError(f"Opcode {opcode:#x} payload too large ({length})")
                fh.write(struct.pack('<IBB', op.delta & 0xFFFFFFFF, opcode, length))
                fh.write(data)
        for trig in builder.triggers:
            fh.write(struct.pack('<IHB', trig.delta & 0xFFFFFFFF,
                                 trig.ssf_index & 0xFFFF,
                                 trig.voice & 0xFF))
            fh.write(b'\x00')  # padding


def write_json(builder: Beta99Builder, out_path: Path) -> None:
    bundle = {
        'ssfs': [
            {
                'hashid': ssf.hashid,
                'duration': ssf.duration,
                'ops': [{'delta': op.delta, 'opcode': op.opcode, 'data': list(op.data)}
                        for op in ssf.ops],
            }
            for ssf in builder.ssfs
        ],
        'triggers': [
            {'delta': trig.delta, 'ssf_index': trig.ssf_index, 'voice': trig.voice}
            for trig in builder.triggers
        ],
    }
    out_path.write_text(json.dumps(bundle, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser(description='Pack desidulate SSFs into beta99 bundles')
    parser.add_argument('--ssf', required=True, type=Path, help='Path to *.ssf.zst file')
    parser.add_argument('--log', required=True, type=Path, help='Path to *.log.zst file')
    parser.add_argument('--out', required=True, type=Path, help='Output .b99 path')
    parser.add_argument('--json', type=Path, help='Optional JSON bundle for debugging')
    parser.add_argument('--max-ops', type=int, default=DEFAULT_MAX_OPS_PER_SSF,
                        help='Maximum operations per SSF before splitting (default: %(default)s)')
    args = parser.parse_args()

    builder = Beta99Builder(args.ssf, args.log, max_ops_per_ssf=args.max_ops)
    write_beta99(builder, args.out)
    if args.json:
        write_json(builder, args.json)
    print(f"beta99 bundle written to {args.out} | SSFs: {len(builder.ssfs)} | Triggers: {len(builder.triggers)}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
