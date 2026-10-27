#!/bin/bash
# Build every test program into a memory image for a memory of MEM_BYTES (default 32 KB: one
# thread's part of memory with 2 threads, see THREAD_MEM_SZ in sys_defs.svh).
#   usage: build_images.sh <out_dir> [mem_bytes]
# Course programs (*.s, *.c), FP kernels (fp/*.c) and atomic kernels (atomic/*.c, at -O0 and -O2).
# The linker scripts are copies of linker.lds / aslinker.lds with LENGTH = mem_bytes, so a C
# program's stack starts at the top of that memory (.s programs hard-code their own addresses,
# all below 32 KB). Image = objcopy binary, one hex byte per line, padded to 64 KB (what
# cpu_test +program= loads); a program bigger than mem_bytes is an error.
set -u
P=$(cd "$(dirname "$0")" && pwd)
O=${1:?usage: build_images.sh <out_dir> [mem_bytes]}
MEM=${2:-32768}
GCC=riscv64-unknown-elf-gcc
mkdir -p "$O"
LEN=$(printf '0x%x' "$MEM")
sed "s/LENGTH = 0x10000/LENGTH = $LEN/" "$P/linker.lds"   > "$O/linker.lds"
sed "s/LENGTH = 0x10000/LENGTH = $LEN/" "$P/aslinker.lds" > "$O/aslinker.lds"
grep -q "LENGTH = $LEN" "$O/linker.lds" && grep -q "LENGTH = $LEN" "$O/aslinker.lds" || { echo "linker script edit failed"; exit 1; }

BASE="-mno-relax -mabi=ilp32 -nostartfiles -mstrict-align"
fail=0
image() {   # <name>: elf -> mem, checking the size
  riscv64-unknown-elf-objcopy -O binary "$O/$1.elf" "$O/$1.bin"
  python3 -I -c "
import sys
d = open(sys.argv[1], 'rb').read()
if len(d) > int(sys.argv[3]): sys.exit('%s: %d bytes, more than %s' % (sys.argv[1], len(d), sys.argv[3]))
d += bytes(65536 - len(d))
open(sys.argv[2], 'w').write(''.join('%02x\n' % b for b in d))" "$O/$1.bin" "$O/$1.mem" "$MEM" || fail=1
}

for f in "$P"/*.s; do
  n=$(basename "$f" .s); [ "$n" = crt ] && continue
  $GCC $BASE -march=rv32im -Wno-main -T "$O/aslinker.lds" "$f" -o "$O/$n.elf" || { fail=1; continue; }
  image "$n"
done
for f in "$P"/*.c; do
  n=$(basename "$f" .c)
  $GCC $BASE -march=rv32im -std=gnu11 -mno-div -O0 -T "$O/linker.lds" "$P/crt.s" "$f" -o "$O/$n.elf" -lm || { fail=1; continue; }
  image "$n"
done
for opt in O0 O2; do
  for f in "$P"/fp/*.c; do
    n=$(basename "$f" .c)_$opt
    $GCC $BASE -march=rv32imaf -std=gnu11 -mno-div -fno-math-errno -$opt -T "$O/linker.lds" "$P/crt.s" "$f" -o "$O/$n.elf" || { fail=1; continue; }
    image "$n"
  done
  for f in "$P"/atomic/*.c; do
    n=$(basename "$f" .c)_$opt
    $GCC $BASE -march=rv32imaf -std=gnu11 -fno-math-errno -$opt -T "$O/linker.lds" "$P/crt.s" "$f" -o "$O/$n.elf" || { fail=1; continue; }
    image "$n"
  done
done
echo "$(ls "$O"/*.mem | wc -l | tr -d ' ') images in $O for a $MEM-byte memory"
exit $fail
