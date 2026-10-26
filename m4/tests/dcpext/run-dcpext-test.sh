#!/bin/bash
# m4_dcpext_fixup() (dcp-fdt.c) on real ADTs and board.dtb fixtures: PASS/FAIL per case, non-zero exit on any failure.
#   run-dcpext-test.sh TREE ADT [ADT-701] WORK
# TREE = a source checkout (src/, sysinc/); ADT = this Mac's ADT (the 25G83 one the static dcpext memory was read from);
# ADT-701 = optional second ADT (macOS 25G241 stub, mBoot-18000.161.10.701.1: the same segments).
# Needs gcc (aarch64 or any host: the code reads the ADT through the double) and dtc.
set -e
TREE=$(cd "${1:?usage: run-dcpext-test.sh TREE ADT [ADT-701] WORK}" && pwd); ADT=$(readlink -f "${2:?ADT}")
if [ $# -ge 4 ]; then ADT701=$(readlink -f "$3"); W=$4; else ADT701=; W=${3:?WORK}; fi
HERE=$(cd "$(dirname "$0")" && pwd); T=$HERE/..
W=$(mktemp -d "$W/dcpext.XXXX")
INC="-I$TREE/src -idirafter $TREE/src/libfdt -idirafter $TREE/sysinc -idirafter $TREE/src/standalone"
F="-w -O1 -g -DM4_LOADER -DM4_DCP_STAGE=3 ${SAN:+-fsanitize=address,undefined -fno-sanitize-recover=undefined}"
for f in dcp-fdt fdt-check; do gcc -c $F $INC $TREE/src/standalone/$f.c -o $W/$f.o; done
for f in fdt fdt_ro fdt_rw fdt_wip fdt_empty_tree fdt_strerror fdt_addresses fdt_sw; do gcc -c -w -I$TREE/src -I$TREE/src/libfdt $TREE/src/libfdt/$f.c -o $W/lf-$f.o; done
gcc -c -w $INC $T/tools/adt-double.c -o $W/adt-double.o
cat > $W/stubs.c <<'C'
#include <stdint.h>
struct m4_display_handoff;
int m4_display_handoff_accepted(const struct m4_display_handoff *h) { return !!h; }
C
gcc -c -w $W/stubs.c -o $W/stubs.o
gcc -c $F $INC $HERE/dcpext-test.c -o $W/test.o
gcc -no-pie ${SAN:+-fsanitize=address,undefined} $W/*.o -o $W/dcpext-test
for n in placeholder static none ram; do dtc -q -I dts -O dtb -o $W/$n.dtb $HERE/fixture-$n.dts; done
mkdir -p $W/adt; python3 -I $HERE/make-variants.py "$ADT" $W/adt
export ASAN_OPTIONS=detect_leaks=0   # the test binary leaks its inputs on purpose
pass=0; fail=0
case_() {   # case_ NAME ADT DTB EXPECT
  if $W/dcpext-test "$2" "$3" "$W/out-$1.dtb" "$4" 2> $W/$1.log; then echo "PASS $1"; pass=$((pass+1)); else echo "FAIL $1"; cat $W/$1.log; fail=$((fail+1)); fi
}
case_ filled               "$ADT"                    $W/placeholder.dtb units:0,0
[ -z "$ADT701" ] || case_ filled-701 "$ADT701" $W/placeholder.dtb units:0,0
case_ static-untouched     "$ADT"                    $W/static.dtb      units:2,2
case_ no-dcpext-nodes      "$ADT"                    $W/none.dtb        none
case_ unknown-firmware     $W/adt/fw-unknown.adt     $W/placeholder.dtb units:3,3
case_ unit1-not-running    $W/adt/not-running.adt    $W/placeholder.dtb units:0,4
case_ segment-names        $W/adt/names.adt          $W/placeholder.dtb units:4,0
case_ remap-out-of-range   $W/adt/remap.adt          $W/placeholder.dtb units:4,0
case_ unaligned-phys       $W/adt/unaligned.adt      $W/placeholder.dtb units:0,4
case_ overlapping-segments $W/adt/overlap.adt        $W/placeholder.dtb units:4,0
case_ phys-wraps           $W/adt/phys-top.adt       $W/placeholder.dtb units:4,0
case_ region-in-ram        "$ADT"                    $W/ram.dtb         units:5,5
# a second run over its own output must change nothing: the filled nodes are no longer placeholders
cp $W/out-filled.dtb $W/again.dtb
case_ second-run           "$ADT"                    $W/again.dtb       units:2,2
cmp -s $W/out-filled.dtb $W/out-second-run.dtb && { echo "PASS second-run-identical"; pass=$((pass+1)); } || {
  # the record is rewritten (<2 2>), so only the tree minus /chosen m4,dcpext may match
  dtc -q -I dtb -O dts $W/out-filled.dtb | grep -v 'm4,dcpext' > $W/a.dts; dtc -q -I dtb -O dts $W/out-second-run.dtb | grep -v 'm4,dcpext' > $W/b.dts
  cmp -s $W/a.dts $W/b.dts && { echo "PASS second-run-identical (apart from the record)"; pass=$((pass+1)); } || { echo "FAIL second-run-identical"; diff $W/a.dts $W/b.dts | head; fail=$((fail+1)); }; }
# SAN=1 builds with ASan/UBSan; FUZZ=N adds N ADTs with random bytes changed in the dcpext properties
if [ -n "$FUZZ" ]; then
  python3 -I $HERE/fuzz-adt.py "$ADT" $W/adt $FUZZ
  fz=0
  for f in $W/adt/fuzz-*.adt; do
    if $W/dcpext-test "$f" $W/placeholder.dtb $W/out-fuzz.dtb any 2> $W/fuzz.log; then fz=$((fz+1)); else echo "FAIL $f"; cat $W/fuzz.log; fail=$((fail+1)); fi
  done
  echo "PASS fuzz: $fz ADTs"; pass=$((pass+fz))
fi
echo "dcpext: $pass passed, $fail failed (work: $W)"
[ $fail = 0 ]
