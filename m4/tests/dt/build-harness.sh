#!/bin/bash
# Build the device-tree harness (rev12-harness.c: the real fix-up chain on a real ADT) against one loader tree.
#   build-harness.sh TREE OUT-BINARY BOARD.DTB WORK-DIR
# TREE = a source checkout (needs src/, sysinc/). Linked at 0x10005280000 so that the addresses the emitters compare
# lie where the loader's do. Host compiler (gcc, aarch64 Linux).
# src/ is -I, not -idirafter: a host header of the same name (cdparanoia ships /usr/include/utils.h) must not win.
set -e
TREE=$(cd "${1:?usage: build-harness.sh TREE OUT BOARD.DTB WORK}" && pwd); OUT=$(readlink -f "${2:?OUT}"); BOARD=$(readlink -f "${3:?BOARD.DTB}")
HERE=$(cd "$(dirname "$0")" && pwd); T=$HERE/../tools; W=$(mktemp -d "${4:?WORK}/harness-obj.XXXX")
F="-w -O1 -g -DM4_LOADER -DM4_MEDIA_TEST -DM4_DCP_STAGE=3"
INC="-I$TREE/src -idirafter $TREE/src/libfdt -idirafter $TREE/sysinc -idirafter $TREE/src/standalone"
for f in bt-boot-admission bt-boot-current-adt direct-smp dart-capture dart-tables dart-preserve display-ownership metadata memory dcp-fdt media-probe fixups display-handoff-native fdt-check; do
  gcc -c $F $INC $TREE/src/standalone/$f.c -o $W/$f.o; done
# cpu-idle-regs.c reads MPIDR/MIDR/CurrentEL: replaced by the values of this SoC (host build)
python3 -I - "$TREE" "$W" <<'PY'
import sys
t, w = sys.argv[1], sys.argv[2]
s = open(t + "/src/standalone/cpu-idle-regs.c").read()
s = s.replace("mrs(MPIDR_EL1)", "0x80000100UL").replace("mrs(MIDR_EL1)", "0x611f0551UL")
s = s.replace('__asm__ volatile("mrs %0, CurrentEL" : "=r"(current_el));', "current_el = 8;")
open(w + "/cpu-idle-regs-host.c", "w").write(s)
PY
gcc -c $F $INC $W/cpu-idle-regs-host.c -o $W/cpu-idle-regs.o
gcc -c -w -O1 -DM4_LOADER -DM4_MEDIA_TEST $INC $TREE/src/standalone/display-handoff.c -o $W/display-handoff.o
for f in fdt fdt_ro fdt_rw fdt_wip fdt_empty_tree fdt_strerror fdt_addresses fdt_sw; do gcc -c -w -I$TREE/src -I$TREE/src/libfdt $TREE/src/libfdt/$f.c -o $W/lf-$f.o; done
gcc -c -w $INC $T/adt-double.c -o $W/adt-double.o
cat > $W/ram-stubs.c <<'C'
#include <string.h>
#include <stddef.h>
#include <stdint.h>
void memcpy64(void *d, void *s, size_t n) { memcpy(d, s, n); }
void memcpy8(void *d, void *s, size_t n) { memcpy(d, s, n); }
void memset64(void *d, uint64_t v, size_t n) { memset(d, (int)v, n); }
void memset8(void *d, uint8_t v, size_t n) { memset(d, v, n); }
C
gcc -c -O1 $W/ram-stubs.c -o $W/ram-stubs.o
gcc -c $F $INC "-DTEMPLATE_PATH=\"$BOARD\"" $HERE/rev12-harness.c -o $W/harness.o
gcc -no-pie -Wl,-Ttext-segment=0x10005280000 $W/*.o -o $OUT
rm -rf "$W"
echo "built $OUT from $TREE"
