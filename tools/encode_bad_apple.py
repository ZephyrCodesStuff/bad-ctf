#!/usr/bin/env python3
"""Encode Bad Apple!! video (input.mkv) + chiptune (music.asm) into a single W25Q16 .bin (YAV1).

Layout:
  [0..28]  Header (28 bytes, little-endian):
             magic[4]      = b"YAV1"
             width: u16    = 160
             height: u16   = 120
             fps: u16      = 10
             music_us: u16 = microseconds per music tick
             num_frames: u32
             music_offset: u32
             music_len: u32
             video_offset: u32
  [music_offset..] RLE music stream: pairs of (midi_note: u8, dur_ticks: u8)
                   note == 0xFE means silence/rest.
  [video_offset..] Video frames: each frame is [comp_len: u16] + PackBits-RLE(cur XOR prev).
"""

import re
import struct
import subprocess
from pathlib import Path

INPUT_MKV = Path("/Users/zeph/Downloads/BadAppleFloppy-master/encoder/assets/input.mkv")
MUSIC_ASM = Path("/Users/zeph/Downloads/BadAppleFloppy-master/floppy/kernel/music.asm")
OUT_BIN = Path(__file__).resolve().parent.parent / "res" / "bad_apple_av.bin"

W, H = 160, 120
FPS = 10
TARGET_DURATION_S = 219.0  # Standard Bad Apple!! PV duration (3m39s)


def parse_music_rle(path: Path) -> tuple[bytes, int]:
    txt = path.read_text()
    start = txt.index("ba_music:")
    body = txt[start:]
    end = body.find("ba_music_end:")
    if end != -1:
        body = body[:end]
    vals = [int(x, 16) for x in re.findall(r"0([0-9a-fA-F]{2})h", body)]

    events = []
    for b in vals:
        if b == 0xFF:
            if events:
                events[-1][1] += 1
            else:
                events.append([0xFE, 1])
        else:
            events.append([b, 1])

    out = bytearray()
    for note, dur in events:
        while dur > 255:
            out.append(note)
            out.append(255)
            dur -= 255
        out.append(note)
        out.append(dur)

    total_ticks = len(vals)
    return bytes(out), total_ticks


def pack_1bit(raw: bytes) -> bytes:
    n = len(raw) // 8
    out = bytearray(n)
    for i in range(n):
        b = 0
        base = i * 8
        if raw[base] >= 128: b |= 0x80
        if raw[base + 1] >= 128: b |= 0x40
        if raw[base + 2] >= 128: b |= 0x20
        if raw[base + 3] >= 128: b |= 0x10
        if raw[base + 4] >= 128: b |= 0x08
        if raw[base + 5] >= 128: b |= 0x04
        if raw[base + 6] >= 128: b |= 0x02
        if raw[base + 7] >= 128: b |= 0x01
        out[i] = b
    return bytes(out)


def packbits_encode(buf: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(buf)
    while i < n:
        run = 1
        while i + run < n and buf[i + run] == buf[i] and run < 128:
            run += 1
        if run >= 3:
            out.append(257 - run)
            out.append(buf[i])
            i += run
        else:
            lit_start = i
            lit_len = 0
            while i < n and lit_len < 128:
                r = 1
                while i + r < n and buf[i + r] == buf[i] and r < 3:
                    r += 1
                if r >= 3 and lit_len > 0:
                    break
                i += 1
                lit_len += 1
            out.append(lit_len - 1)
            out.extend(buf[lit_start : lit_start + lit_len])
    return bytes(out)


def main() -> None:
    music_bytes, total_ticks = parse_music_rle(MUSIC_ASM)
    music_us = round((TARGET_DURATION_S * 1_000_000) / total_ticks)
    print(f"Music: {total_ticks} ticks, {len(music_bytes)} RLE bytes, {music_us} us/tick")

    # Rescale input.mkv (324.18s) to match TARGET_DURATION_S (219.0s) at FPS
    pts_scale = TARGET_DURATION_S / 324.177
    frame_bytes = (W * H) // 8

    proc = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-i",
            str(INPUT_MKV),
            "-vf",
            f"setpts={pts_scale:.6f}*PTS,fps={FPS},scale={W}:{H}",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "gray",
            "-",
        ],
        stdout=subprocess.PIPE,
    )

    prev = bytes(frame_bytes)
    video_blob = bytearray()
    num_frames = 0

    while True:
        raw = proc.stdout.read(W * H)
        if len(raw) < W * H:
            break
        cur = pack_1bit(raw)
        xor = bytes(a ^ b for a, b in zip(prev, cur))
        comp = packbits_encode(xor)
        video_blob.extend(struct.pack("<H", len(comp)))
        video_blob.extend(comp)
        prev = cur
        num_frames += 1

    proc.wait()

    music_offset = 28
    video_offset = music_offset + len(music_bytes)
    header = struct.pack(
        "<4sHHHHIIII",
        b"YAV1",
        W,
        H,
        FPS,
        music_us,
        num_frames,
        music_offset,
        len(music_bytes),
        video_offset,
    )

    total = header + music_bytes + video_blob
    OUT_BIN.write_bytes(total)
    print(
        f"Wrote {OUT_BIN}: {len(total):,} bytes ({len(total)/1024:.1f} KiB, {len(total)/(1024*1024):.2f} MiB) "
        f"— {num_frames} frames @ {FPS}fps ({num_frames/FPS:.1f}s)"
    )
    if len(total) > 2 * 1024 * 1024:
        print("WARNING: exceeds W25Q16 2 MiB capacity!")


if __name__ == "__main__":
    main()
