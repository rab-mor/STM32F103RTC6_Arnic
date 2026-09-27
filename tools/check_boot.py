#!/usr/bin/env python3
"""check_boot.py - the bootloader must not reach into the application.

The bootloader (Core/MySrc/f1_boot.c) is linked into the application's ELF at
0x08000000 and copies new application images over 0x08004000 onwards. If any
of its code called something outside its own 16 KB (memcpy, a CMSIS helper
the compiler did not inline, libgcc), that call would jump into flash it is
busy erasing. This script disassembles the .boot section of a built ELF and
fails if any branch leaves it.

usage: python3 tools/check_boot.py Debug/<project>.elf  [objdump]
       (objdump defaults to arm-none-eabi-objdump; CubeIDE ships one under
        plugins/...gnu-tools-for-stm32.../tools/bin/)
"""
import re, subprocess, sys

elf = sys.argv[1]
objdump = sys.argv[2] if len(sys.argv) > 2 else "arm-none-eabi-objdump"

hdrs = subprocess.run([objdump, "-h", elf], capture_output=True, text=True, check=True).stdout
m = re.search(r"^\s*\d+\s+\.boot\s+([0-9a-f]+)\s+([0-9a-f]+)", hdrs, re.M)
if not m:
    sys.exit("no .boot section: is f1_boot.c in the build and the linker script updated?")
size, start = int(m.group(1), 16), int(m.group(2), 16)
end = start + size

dis = subprocess.run([objdump, "-d", "-j", ".boot", elf], capture_output=True, text=True, check=True).stdout
bad = []
branches = 0
for line in dis.splitlines():
    mm = re.match(r"\s*([0-9a-f]+):\s+(?:[0-9a-f]{4}\s?){1,2}\s+(\S+)\s+(.*)", line)
    if not mm:
        continue
    addr, op, args = int(mm.group(1), 16), mm.group(2), mm.group(3)
    if re.fullmatch(r"(b|bl|b\.w|b\.n|b[a-z]{2}(\.[nw])?|cbn?z)", op):
        t = re.search(r"\b([0-9a-f]{7,8})\b", args)
        if t:
            branches += 1
            tgt = int(t.group(1), 16)
            if not (start <= tgt < end):
                bad.append("%08x: %s %s  -> outside .boot" % (addr, op, args))
    elif op in ("blx",):
        bad.append("%08x: %s %s  -> indirect call" % (addr, op, args))

print(".boot: 0x%08x..0x%08x (%d bytes), %d direct branches checked" % (start, end, size, branches))
if size > 0x4000:
    bad.append(".boot is larger than the 16 KB BOOT region")
if bad:
    print("FAIL:\n  " + "\n  ".join(bad))
    sys.exit(1)
print("OK: nothing in the bootloader leaves it")
