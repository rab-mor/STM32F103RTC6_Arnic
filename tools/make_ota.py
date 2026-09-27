#!/usr/bin/env python3
"""make_ota.py - build a firmware update package for the H7 or an F103 board.

The same file is in all three STM32 projects. It reads the ELF files CubeIDE
builds (Debug/ or Release/), takes only what goes into each image's flash
slot, adds the header the target checks, and prints the manifest to send
(MQTT command topic or the web dashboard) once the file is on a web server.

  H7 (both cores in one package; the H7 installs it by bank swap):
    python3 tools/make_ota.py h7 --cm7 CM7/Debug/STM32H755_firmware_CM7.elf \\
                                 --cm4 CM4/Debug/STM32H755_firmware_CM4.elf \\
                                 --version 1.4.2 -o arnic-h7-1.4.2.bin
  Main-board F103 / an expansion module:
    python3 tools/make_ota.py f103 --elf Debug/STMF103RTC6_Firmware.elf --version 1.1.0 -o arnic-f103-1.1.0.bin
    python3 tools/make_ota.py exp  --elf Debug/Arnic_ExpansionModule.elf --version 1.1.0 -o arnic-exp-1.1.0.bin
  Add --url https://server/path/file.bin to have the manifest printed with it.

Formats: Common/Inc/h7_image.h (H7) and Core/MyHeaders/f1_image.h (F103s).
The F1 package leaves out the bootloader (0x08000000-0x08003FFF): it is only
ever written with the debugger. On an F1, keep FW_VERSION_MAJOR/MINOR in
board_config.h equal to --version's first two numbers: that is what the board
reports once it runs the new image.

Python 3.6+, standard library only.
"""
import argparse, hashlib, json, os, struct, sys, time, zlib

H7_MAGIC, H7_HDR_VERSION = 0x4D493748, 1
F1_MAGIC, F1_HDR_VERSION = 0x4D493146, 1
F1_BOARD = {"exp": 1, "f103": 2}

H7_CM7 = (0x08000000, 0x80000)          # slot base, max size
H7_CM4 = (0x08080000, 0x80000 - 0x400)  # the last 1 KB is the update header
F1_APP = (0x08004000, 0x1D800)


def load_elf(path):
    """[(paddr, bytes)] for every PT_LOAD segment with file contents."""
    data = open(path, "rb").read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        sys.exit("%s: not a 32-bit little-endian ELF" % path)
    phoff, = struct.unpack_from("<I", data, 0x1C)
    phentsize, phnum = struct.unpack_from("<HH", data, 0x2A)
    segs = []
    for i in range(phnum):
        p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align = \
            struct.unpack_from("<IIIIIIII", data, phoff + i * phentsize)
        if p_type == 1 and p_filesz > 0:
            segs.append((p_paddr, data[p_offset:p_offset + p_filesz]))
    return segs


def slot_image(segs, base, size, name, ignore=()):
    """The bytes of [base, base+size), 0xFF where nothing is loaded, trimmed
    after the last loaded byte. Anything loaded partly outside is an error;
    segments wholly outside are someone else's (RAM, the other core, the
    bootloader) and are skipped."""
    img = bytearray(b"\xff" * size)
    end = 0
    for addr, blob in segs:
        a_end = addr + len(blob)
        if a_end <= base or addr >= base + size:
            continue
        if addr < base or a_end > base + size:
            if any(lo <= addr < hi for lo, hi in ignore):
                continue
            sys.exit("%s: segment 0x%08X..0x%08X runs outside its slot 0x%08X..0x%08X" %
                     (name, addr, a_end, base, base + size))
        img[addr - base:a_end - base] = blob
        end = max(end, a_end - base)
    if end == 0:
        sys.exit("%s: nothing is linked at 0x%08X; wrong ELF or old linker script?" % (name, base))
    sp, pc = struct.unpack_from("<II", img, 0)
    if not (pc & 1 and base < pc < base + size and 0x10000000 <= sp <= 0x38010000):
        sys.exit("%s: no vector table at 0x%08X (SP 0x%08X, reset 0x%08X)" % (name, base, sp, pc))
    return bytes(img[:end])


def parse_version(v):
    parts = [int(x) for x in v.split(".")]
    while len(parts) < 3:
        parts.append(0)
    if len(parts) != 3 or any(not 0 <= x <= 255 for x in parts):
        sys.exit("--version must look like 1.4.2 (each part 0..255)")
    return (parts[0] << 16) | (parts[1] << 8) | parts[2]


def crc(b):
    return zlib.crc32(b) & 0xFFFFFFFF


def build_h7(args):
    cm7 = slot_image(load_elf(args.cm7), *H7_CM7, name="CM7")
    cm4 = slot_image(load_elf(args.cm4), *H7_CM4, name="CM4")
    tag = args.version.encode()[:27].ljust(28, b"\0")
    head = struct.pack("<IHHIIIIII28s", H7_MAGIC, H7_HDR_VERSION, 0, parse_version(args.version),
                       len(cm7), crc(cm7), len(cm4), crc(cm4), int(time.time()), tag)
    assert len(head) == 60
    head += struct.pack("<I", crc(head))
    print("CM7 image %d bytes, CM4 image %d bytes" % (len(cm7), len(cm4)))
    return head + cm7 + cm4


def build_f1(args):
    # The bootloader sits below the app slot in the same ELF: skip it.
    app = slot_image(load_elf(args.elf), *F1_APP, name=args.target,
                     ignore=[(0x08000000, 0x08004000)])
    head = struct.pack("<IHBBIIIII", F1_MAGIC, F1_HDR_VERSION, F1_BOARD[args.target], 0,
                       parse_version(args.version), len(app), crc(app), int(time.time()), 0)
    assert len(head) == 28
    head += struct.pack("<I", crc(head))
    print("application image %d bytes" % len(app))
    return head + app


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("target", choices=["h7", "f103", "exp"])
    ap.add_argument("--cm7"); ap.add_argument("--cm4"); ap.add_argument("--elf")
    ap.add_argument("--version", required=True)
    ap.add_argument("--addr", type=int, default=0, help="exp: module address 0..15 for the manifest")
    ap.add_argument("--url", default="https://<server>/<file>")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    if args.target == "h7":
        if not (args.cm7 and args.cm4):
            sys.exit("h7 needs --cm7 and --cm4")
        pkg = build_h7(args)
    else:
        if not args.elf:
            sys.exit("%s needs --elf" % args.target)
        pkg = build_f1(args)

    open(args.output, "wb").write(pkg)
    manifest = {"type": "ota", "target": args.target, "version": args.version,
                "size": len(pkg), "sha256": hashlib.sha256(pkg).hexdigest(), "url": args.url}
    if args.target == "exp":
        manifest["addr"] = args.addr
    print("wrote %s (%d bytes)" % (args.output, len(pkg)))
    print("manifest:")
    print(json.dumps(manifest))


if __name__ == "__main__":
    main()
