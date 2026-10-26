#!/bin/bash
# The loader test suite: one command, PASS/FAIL per step, non-zero exit if any step fails.
#
#   INPUTS=/path/to/inputs PRIVATE=/path/to/private m4/check.sh [step ...]
#
# Steps (default: all, in this order):
#   build     build.sh + layout check, compared with the reference hashes (m1n1.bin, level1.bin, owned layout, offsets, inputs)
#   objects   every object file, stripped of debug info/.comment/.note*, compared byte for byte with the reference list
#   qemu      the Level 1 guard (EL2 + EL1 flavours) and the shim + blob + guard (2 blob placements) under QEMU at EL2
#   e2e       the Level 1 blob end to end with the model HAL: this Mac's ADT, a real ext4 image per scenario (37 cases)
#   decision  loader-decision (the production policy on an ext4 image): 38 fixtures + 3 two-boot one-shot sequences + 3 board.dtb-window fixtures, --window-dtb
#   harness   the real DT fix-up chain on the real ADT (rev12-harness): 8 scenarios + the /chosen m4,board-dtb-max edit
#   pmp       rev 13 m4_pmp_publish() on the real ADT and a final tree (pmp-test)
#   rev14     rev 14: aop-dapf.c on fake MMIO and the real ADT, the AOP segment iommu-addresses, the Bluetooth admission fix
#             (the m4.13 two-port PCIe node), the /chosen records m4,dapf-*, m4,bt-admission, m4,boot-timing-v1
#   dcpext    m4_dcpext_fixup() (the external DCPs' firmware memory) on the real ADT: placeholders filled with the values of the static
#             dtsi, refusals, fuzzed ADTs (m4/tests/dcpext). Not part of the reference file; skipped before the fix-up exists.
#   audit     the per-unit checks of m4/tools (audit-tree.py on the checkout, gates-unit.py check, usb-contract-id.py) against the ADT;
#             skipped if those tools are not in the repo. Not in the default run when PRIVATE is unset.
#
# Variables:
#   INPUTS     dir with Image, board.dtb, rescue-initramfs.img, rescue-cmdline.txt, pcie.bin, librust.a   (required)
#   PRIVATE    the kit's private/ dir: private-gates-unit.h and private-level1-install.h (build), private-adt.bin and private-adt-ref.bin
#              (this Mac's Apple device tree: per unit, never in git; qemu, e2e, harness, pmp) and optionally
#              private-reference-revNN.txt (hashes of the harness output trees; revNN as in REFERENCE)
#   SRC        the source checkout (default: the repo this script lives in)
#   VERSION    loader version stamp (default: the reference file's `version`)
#   REFERENCE  reference file (default m4/reference/reference-rev19.txt; rev18/rev17/rev16/rev15/rev14/rev13 for those trees)
#   WORK       scratch directory (default: a new mktemp dir; logs in $WORK/logs, observed values in $WORK/obs)
#   QEMU       qemu-system-aarch64 binary (default: from PATH)
#   ROOT_UUID  only if rescue-cmdline.txt has the @ROOT_UUID@ placeholder
#   REF_SRC    optional: a second checkout (e.g. rev 12) to compare against in `harness` (final trees must be identical)
#   REF_BUILD  optional: the build dir (OUT/source/build) of another build; `objects` prints the object differences to it
#   NATIVE=1   the toolchain is not the container's (e.g. the Arch host: gcc 16.1, binutils 2.46): the bytes of m1n1.bin, level1.bin and
#              the objects are expected to differ, so they are reported but not compared; everything else (sizes, layout offsets,
#              owned-layout hash, layout status, all the tests) must still match
#   KERNEL_DTB optional: the Linux side's real board.dtb; adds a `kernel-dtb` scenario to `harness` (no reference value; it must just pass)
set -u
. "$(dirname "$0")/env.sh"
SRC=${SRC:-$M4_ROOT}; INPUTS=${INPUTS:-}; PRIVATE=${PRIVATE:-}
REFERENCE=${REFERENCE:-$M4_HERE/reference/reference-rev19.txt}
[ -f "$REFERENCE" ] || { echo "no reference file: $REFERENCE" >&2; exit 2; }
ref() { awk -v k="$1" '$1 == k { sub(/^[^ ]+ +/, ""); print; exit }' "$REFERENCE"; }
VERSION=${VERSION:-$(ref version)}
WORK=${WORK:-$(mktemp -d "${TMPDIR:-/tmp}/m4-check.XXXXXX")}; mkdir -p "$WORK/logs" "$WORK/obs"; WORK=$(cd "$WORK" && pwd)
QEMU=${QEMU:-qemu-system-aarch64}; export QEMU
B=$WORK/build; S=$B/source/src/standalone
[ -n "$INPUTS" ] && INPUTS=$(cd "$INPUTS" && pwd); [ -n "$PRIVATE" ] && PRIVATE=$(cd "$PRIVATE" && pwd)
SRC=$(cd "$SRC" && pwd)
BOARD=$INPUTS/board.dtb
ADT=${PRIVATE:+$PRIVATE/private-adt.bin}; ADTREF=${PRIVATE:+$PRIVATE/private-adt-ref.bin}
TREE=$B/source      # the tests compile the build copy: it has the private headers (gates-unit.h, level1-install.h) next to the sources
OBS=/dev/null
obs() { printf '%s %s\n' "$1" "$2" >> "$OBS"; }
sha() { sha256sum "$1" | cut -d' ' -f1; }
size() { stat -c %s "$1"; }
need_private() { [ -n "$PRIVATE" ] && [ -f "$ADT" ] && [ -f "$ADTREF" ] || { echo "PRIVATE must name a directory with private-adt.bin and private-adt-ref.bin (this Mac's ADT; the kit's private/ folder)"; return 1; }; }
need_built() { [ -f "$B/result.json" ] || { echo "run the build step first (or all steps)"; return 1; }; }
FAILS=0; SKIPS=0; PASSES=0

# ---- compare what a step observed with the reference file -------------------------------------------------
verify() {   # verify STEP: every observed key must equal the reference value; every reference key of the step must be observed
  local step=$1 rc=0 k v want
  while read -r k v; do
    if [ "${NATIVE:-0}" = 1 ]; then case $k in build.m1n1.bin.sha256|build.level1.bin.sha256|objects.*) echo "  note: $k = $v (NATIVE=1: toolchain-bound, not compared)"; continue;; esac; fi
    want=$(ref "$k")
    if [ -z "$want" ] && ! grep -q "^$k " "$REFERENCE"; then echo "  note: $k = $v (not in the reference)"; continue; fi
    [ "$v" = "$want" ] || { echo "  MISMATCH $k: got '$v', reference '$want'"; rc=1; }
  done < "$WORK/obs/$step.txt"
  while read -r k _; do
    if [ "${NATIVE:-0}" = 1 ]; then case $k in build.m1n1.bin.sha256|build.level1.bin.sha256|objects.*) continue;; esac; fi
    case $k in "$step".*|"$step:"*) grep -q "^$k " "$WORK/obs/$step.txt" || { echo "  MISSING $k (in the reference, not observed)"; rc=1; };; esac
  done < <(grep -v '^#' "$REFERENCE")
  return $rc
}
step() {
  local name=$1; shift
  case " $WANT " in *" all "*|*" $name "*) ;; *) return 0;; esac
  : > "$WORK/obs/$name.txt"; OBS=$WORK/obs/$name.txt
  local t0=$SECONDS rc=0
  ( set -eu; "step_$name" ) > "$WORK/logs/$name.log" 2>&1; rc=$?
  if [ $rc -eq 0 ]; then verify "$name" >> "$WORK/logs/$name.log" 2>&1; rc=$?; fi
  OBS=/dev/null
  if [ $rc -eq 77 ]; then SKIPS=$((SKIPS+1)); printf 'SKIP  %-9s (%ss)  %s\n' "$name" $((SECONDS-t0)) "$(tail -1 "$WORK/logs/$name.log")"
  elif [ $rc -eq 0 ]; then PASSES=$((PASSES+1)); printf 'PASS  %-9s (%ss)  %s\n' "$name" $((SECONDS-t0)) "$(grep '^SUMMARY ' "$WORK/logs/$name.log" | tail -1 | cut -c9-)"
  else FAILS=$((FAILS+1)); printf 'FAIL  %-9s (%ss)  log: %s\n' "$name" $((SECONDS-t0)) "$WORK/logs/$name.log"; tail -8 "$WORK/logs/$name.log" | sed 's/^/        /'; fi
}

# ---- 1. build + layout check ---------------------------------------------------------------------------------
step_build() {
  [ -n "$INPUTS" ] || { echo "INPUTS not set"; return 1; }
  "$M4_HERE/build.sh" SRC="$SRC" INPUTS="$INPUTS" OUT="$B" VERSION="$VERSION" ${PRIVATE:+PRIVATE="$PRIVATE"}
  python3 -I - "$B" "$INPUTS" >> "$OBS" <<'PY'
import hashlib, json, sys, os
b, inp = sys.argv[1], sys.argv[2]
r = json.load(open(b + '/result.json'))
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
print('build.version', r['version']); print('build.m1n1.bin.sha256', r['m1n1_bin_sha256']); print('build.m1n1.bin.bytes', r['bytes'])
print('build.level1.bin.bytes', r['level1_blob_bytes']); print('build.level1.bin.sha256', sha(b + '/source/build/level1.bin'))
print('build.owned-layout.sha256', r['owned_layout_sha256']); print('build.layout.status', r['status'].split()[0])
for k, v in r['offsets'].items(): print('build.offset.' + k, v)
for k in ('image_sha256', 'initramfs_sha256', 'initramfs_padded_sha256', 'board_dtb_sha256'): print('build.rescue.' + k, r['rescue'][k])
for n in ('pcie.bin', 'librust.a'): print('build.input.' + n + '.sha256', sha(inp + '/' + n))
print('build.payload.rescue-cmdline.txt.sha256', sha(b + '/payloads/rescue-cmdline.txt'))
print('build.payload.pcie.bin.sha256', sha(b + '/payloads/pcie.bin'))
PY
  grep -c 'warning:' "$B/build.log" | sed 's/^/build.warnings /' >> "$OBS"
  cat "$B/layout.log"; echo "SUMMARY $(sha "$B/source/build/m1n1.bin" | cut -c1-12)… $(size "$B/source/build/m1n1.bin") B, layout $(ref build.layout.status)"
}

# ---- 2. object-level comparison ------------------------------------------------------------------------------
step_objects() {
  need_built
  "$M4_HERE/objhash.sh" "$B/source/build" > "$WORK/objects.txt"
  if [ "${NATIVE:-0}" = 1 ]; then
    echo "NATIVE=1: object hashes are toolchain-bound; listing how many differ from the reference (informational)"
    sed 's/  / /' "$WORK/objects.txt" | LC_ALL=C sort > "$WORK/objects.norm"
    grep '^object ' "$REFERENCE" | sed 's/^object //' | LC_ALL=C sort > "$WORK/objects.ref"
    echo "$(comm -13 "$WORK/objects.ref" "$WORK/objects.norm" | wc -l | tr -d ' ') of $(wc -l < "$WORK/objects.norm" | tr -d ' ') objects differ from the reference"
    return 77
  fi
  obs objects.count "$(wc -l < "$WORK/objects.txt" | tr -d ' ')"
  obs objects.listsha256 "$(sha "$WORK/objects.txt")"
  grep '^object ' "$REFERENCE" | sed 's/^object //' | LC_ALL=C sort -k2 > "$WORK/objects.ref"
  sed 's/  / /' "$WORK/objects.txt" > "$WORK/objects.norm"
  if ! diff -u "$WORK/objects.ref" "$WORK/objects.norm" > "$WORK/objects.diff"; then echo "objects differ from the reference (- reference, + this build):"; grep '^[-+][0-9a-f]' "$WORK/objects.diff" | head -40; return 1; fi
  if [ -n "${REF_BUILD:-}" ]; then echo "--- against $REF_BUILD"; "$M4_HERE/objcmp.sh" "$REF_BUILD" "$B/source/build" || true; fi
  echo "SUMMARY $(wc -l < "$WORK/objects.txt" | tr -d ' ') objects identical to the reference"
}

# ---- 3. QEMU: guard and shim ---------------------------------------------------------------------------------
step_qemu() {
  need_built; need_private; need qemu-system-aarch64 || need "$QEMU"
  echo "qemu: $("$QEMU" --version | head -1)"
  bash "$M4_HERE/tests/guard/run-guard-test.sh" "$TREE" "$WORK" | tee "$WORK/guard.out" | tail -4
  grep -q GUARD_TEST_PASS "$WORK/guard.out"
  obs qemu.guard EL2+EL1:PASS
  for fl in EL2 EL1; do obs qemu.guard.$fl "$(sed -n "s/^L1 guard test $fl: checks=0x\([0-9a-f]*\) fails=0x\([0-9a-f]*\).*/checks=\1,fails=\2/p" "$WORK/guard-test/guard-$fl.log")"; done
  bash "$M4_HERE/tests/shim/run-shim-test.sh" "$B/source" "$WORK" "$ADT" "$BOARD" | tee "$WORK/shim.out" | grep 'SHIM_TEST_\|^=====' || true
  for pad in 0 49152; do
    grep -q "SHIM_TEST_PASS (pad $pad)" "$WORK/shim.out"; obs qemu.shim.pad$pad PASS
    obs qemu.shim.pad$pad.checks "$(sed -n 's/^L1 shim test: checks=0x\([0-9a-f]*\) fails=0x\([0-9a-f]*\).*/checks=\1,fails=\2/p' "$WORK/shim-test-pad$pad/shim.log")"
  done
  echo "SUMMARY guard EL2+EL1, shim pad 0 + 49152 (QEMU $("$QEMU" --version | sed -n '1s/.*version \([0-9.]*\).*/\1/p'))"
}

# ---- fixtures shared by e2e and decision ---------------------------------------------------------------------
make_fixtures() {
  FIX=$WORK/fixtures; rm -rf "$FIX"; mkdir -p "$FIX"
  python3 -I "$M4_HERE/tests/l1/make-abi-fixtures.py" "$FIX" "$BOARD" > "$FIX/list.txt"
  for n in 47000 98304 98308; do python3 -I "$M4_HERE/tests/dt/make-pad.py" "$BOARD" "$FIX/pad-$n.dtb" $n > /dev/null; done
  python3 -I "$M4_HERE/tests/l1/make-big-dtb-fixtures.py" "$FIX" big-dtb-47000="$FIX/pad-47000.dtb" big-dtb-98304="$FIX/pad-98304.dtb" over-dtb-98308="$FIX/pad-98308.dtb" > "$FIX/big.txt"
  grep -q 'rc 0' "$FIX/big.txt" && ! grep -q 'rc [1-9]' "$FIX/big.txt"
}
L1_SRCS() { echo "$S/level1-core.c $S/level1-ext4.c $S/level1-sha256.c $TREE/src/libfdt/fdt.c $TREE/src/libfdt/fdt_ro.c $TREE/src/libfdt/fdt_rw.c $TREE/src/libfdt/fdt_wip.c $TREE/src/libfdt/fdt_strerror.c $TREE/src/libfdt/fdt_addresses.c $TREE/src/libfdt/fdt_empty_tree.c $TREE/src/libfdt/fdt_sw.c $S/fdt-check.c"; }

# ---- 4. Level 1 end to end (model HAL) -----------------------------------------------------------------------
step_e2e() {
  need_built; need_private; make_fixtures
  S=$TREE/src/standalone; local T=$M4_HERE/tests/l1 R=$TREE
  HA=$(sed -n 's/^#define L1_HOST_ABI \([0-9]*\)u.*/\1/p' $S/level1-host.h)
  grep -q "long $HA  *.. L1_HOST_ABI" $S/level1-header.S || { echo 'level1-header.S ABI word != L1_HOST_ABI'; return 1; }
  export ASAN_OPTIONS=detect_leaks=0
  gcc -O1 -g -Wall -Wno-unused-function -Wno-unused-variable -Wno-format-truncation -fsanitize=undefined -march=armv8.2-a -DL1_TEST \
      -I$T -I$S -I$R/src -I$R/src/libfdt -o "$WORK/abi-e2e" $T/abi-e2e.c $S/level1-blob.c $S/level1-core.c $S/level1-ext4.c $S/level1-sha256.c \
      $S/level1-ans.c $S/level1-addr.c $R/src/libfdt/fdt.c $R/src/libfdt/fdt_ro.c $R/src/libfdt/fdt_rw.c $R/src/libfdt/fdt_wip.c $R/src/libfdt/fdt_strerror.c \
      $R/src/libfdt/fdt_addresses.c $R/src/libfdt/fdt_empty_tree.c $R/src/libfdt/fdt_sw.c $S/fdt-check.c
  ( cd "$FIX" && "$WORK/abi-e2e" "$ADT" "$BOARD" "$FIX" ) > "$WORK/abi-e2e.out" 2>&1 || { tail -20 "$WORK/abi-e2e.out"; return 1; }
  tail -3 "$WORK/abi-e2e.out"
  local line; line=$(grep '^ALL PASS' "$WORK/abi-e2e.out" | tail -1); [ -n "$line" ]
  obs e2e.result "$(echo "$line" | sed 's/^ALL PASS: //')"
  obs e2e.fail-lines "$(grep -c '^FAIL' "$WORK/abi-e2e.out" || true)"
  echo "SUMMARY $line"
}

# ---- 5. loader-decision --------------------------------------------------------------------------------------
step_decision() {
  need_built; [ -d "$WORK/fixtures" ] || make_fixtures; FIX=$WORK/fixtures; S=$TREE/src/standalone
  gcc -O2 -march=armv8.2-a -Wall -I$S -I$TREE/src -I$TREE/src/libfdt -o "$WORK/loader-decision" "$M4_HERE/tests/l1/loader-decision.c" $(L1_SRCS)
  local pass=0 fail=0 name source reason out
  while read -r name source reason; do
    out=$("$WORK/loader-decision" "$FIX/$name.ext4" 2>/dev/null)
    if [[ "$out" == "source=$source "*"reason=$reason "* ]]; then pass=$((pass+1)); echo "PASS $name -> source=$source reason=$reason"
    else fail=$((fail+1)); echo "FAIL $name: got '$out' want source=$source reason=$reason"; fi
  done <<'CASES'
ok entry default
pending rescue previous-boot-incomplete
pending-empty rescue previous-boot-incomplete
dirty-state rescue ext4-dirty
dirty-recover rescue ext4-dirty
noconf rescue no-loader-conf
loader-abi2 rescue bad-abi
loader-noabi rescue bad-conf
loader-baddefault rescue bad-conf
loader-dupkey rescue bad-conf
no-entry rescue no-entry
entry-abi2 rescue bad-abi
entry-nocmdline rescue bad-conf
entry-longcmdline rescue bad-conf
entry-maxcmdline entry default
entry-nocrc rescue bad-conf
entry-nosize rescue bad-conf
missing-image rescue missing-file
size-image rescue size-mismatch
size-dtb rescue size-mismatch
crc-initrd rescue digest-mismatch
crc-image rescue digest-mismatch
crc-dtb rescue digest-mismatch
bad-image-magic rescue bad-image
image-textoff rescue bad-image
image-window rescue too-large
bad-dtb rescue bad-dtb
truncated-dtb rescue bad-dtb
m1n1-ok entry default
m1n1-crc rescue digest-mismatch
m1n1-size rescue size-mismatch
m1n1-missing rescue missing-file
m1n1-nosize rescue bad-conf
entry-badtype rescue bad-conf
oneshot-spent entry default
oneshot-small entry default
oneshot-badentry entry default
oneshot-pending rescue previous-boot-incomplete
CASES
  # rev 17: the one-shot token over two boots (the tool keeps the token write in memory): entry, kind, oneshot per boot
  local exp got
  while read -r name exp; do
    got=$("$WORK/loader-decision" --boots 2 "$FIX/$name.ext4" 2>/dev/null | sed -E 's/.* entry=([^ ]+) .* kind=([a-z0-9]+) oneshot=([0-9]).*/\1,\2,\3/' | paste -sd' ' -)
    if [ "$got" = "$exp" ]; then pass=$((pass+1)); echo "PASS $name (2 boots) -> $got"; else fail=$((fail+1)); echo "FAIL $name (2 boots): got '$got' want '$exp'"; fi
  done <<'BOOTS'
oneshot-linux e2,linux,1 e1,linux,0
oneshot-m1n1 e2,m1n1,1 e1,linux,0
oneshot-noentry rescue,linux,1 e1,linux,0
BOOTS
  obs decision.cases "$pass/$((pass+fail))"
  # the board.dtb window: default 98,304 B (rev 12 and later); --window-dtb 36864 is rev 11's
  local w tag out2
  for tag in 98304 36864; do
    w=; [ $tag = 36864 ] && w="--window-dtb 36864"
    for name in big-dtb-47000 big-dtb-98304 over-dtb-98308; do
      out2=$("$WORK/loader-decision" $w "$FIX/$name.ext4" 2>/dev/null | awk '{print $1 ":" $3}')
      echo "window $tag: $name -> $out2"
      obs "decision.window$tag.$name" "$out2"
    done
  done
  [ $fail = 0 ]
  echo "SUMMARY $pass/$((pass+fail)) fixtures + 6 window checks"
}

# ---- 6. DT harness: the real fix-up chain on the real ADT ----------------------------------------------------
run_harness() {   # run_harness BIN NAME ADT BOARD [ENV=1]  -> $WORK/h/NAME.{dtb,log}
  local bin=$1 name=$2 adt=$3 board=$4 envv=${5:-}
  if [ -n "$envv" ]; then env "$envv=1" "$bin" "$adt" "$board" "$WORK/h/$name.dtb" 2> "$WORK/h/$name.log" || { cat "$WORK/h/$name.log"; return 1; }
  else "$bin" "$adt" "$board" "$WORK/h/$name.dtb" 2> "$WORK/h/$name.log" || { cat "$WORK/h/$name.log"; return 1; }; fi
}
scenarios() {   # name adt board env
  cat <<EOS
board $ADTREF $BOARD -
pad-47000 $ADTREF $WORK/fixtures/pad-47000.dtb -
pad-62000 $ADTREF $WORK/h/pad-62000.src.dtb -
pad-98304 $ADTREF $WORK/fixtures/pad-98304.dtb -
bt-refused $ADTREF $BOARD M4_BT_REFUSE
dcp-refused $ADTREF $BOARD M4_DCP_REFUSE
usb-drd2 $WORK/h/adt-usb-drd2.bin $BOARD -
avd0 $WORK/h/adt-avd0.bin $BOARD -
admac-leap-ns $WORK/h/adt-admac-leap-ns.bin $BOARD -
EOS
  [ -n "${KERNEL_DTB:-}" ] && echo "kernel-dtb $ADTREF $KERNEL_DTB -"
  return 0
}
step_harness() {
  need_built; need_private; need dtc; [ -d "$WORK/fixtures" ] || make_fixtures
  mkdir -p "$WORK/h"; local T=$M4_HERE/tests
  : > "$WORK/obs/private-harness.txt"
  python3 -I $T/dt/make-pad.py "$BOARD" "$WORK/h/pad-62000.src.dtb" 62000 > /dev/null
  python3 -I $T/tools/mutate-adt.py "$ADTREF" "$WORK/h/adt-usb-drd2.bin" /arm-io/usb-drd2 interrupts
  python3 -I $T/tools/mutate-adt.py "$ADTREF" "$WORK/h/adt-avd0.bin" /arm-io/avd0 reg
  python3 -I $T/tools/mutate-adt.py "$ADTREF" "$WORK/h/adt-admac-leap-ns.bin" /arm-io/admac-leap-ns reg
  bash $T/dt/build-harness.sh "$TREE" "$WORK/h/harness" "$BOARD" "$WORK/h"
  if [ -n "${REF_SRC:-}" ]; then   # a second tree to compare with: copied, with the same private headers
    rm -rf "$WORK/ref-tree"; cp -r "$REF_SRC" "$WORK/ref-tree"
    for f in gates-unit.h level1-install.h; do [ -f "$WORK/ref-tree/src/standalone/$f" ] || cp "$TREE/src/standalone/$f" "$WORK/ref-tree/src/standalone/$f" 2>/dev/null || true; done
    bash $T/dt/build-harness.sh "$WORK/ref-tree" "$WORK/h/harness-ref" "$BOARD" "$WORK/h"
  fi
  local name adt board envv bad=0 rc line
  while read -r name adt board envv; do
    [ "$envv" = - ] && envv=
    if run_harness "$WORK/h/harness" "$name" "$adt" "$board" "$envv"; then rc=0; else rc=1; fi
    line=$(grep -v '^metadata\|^smp\|^regions' "$WORK/h/$name.log" | sed 's/^\[rev11\] //' | tr '\n' ';' | sed 's/;$//; s/ /_/g')
    echo "$name: $line"
    obs "harness.$name" "$line"
    # the output trees depend on the ADT (per unit): their hashes go to the private reference only
    [ $rc = 0 ] && echo "$name $(sha "$WORK/h/$name.dtb")" >> "$WORK/obs/private-harness.txt"
    if [ -n "${REF_SRC:-}" ]; then
      run_harness "$WORK/h/harness-ref" "ref-$name" "$adt" "$board" "$envv" || true
      dtc -I dtb -O dts "$WORK/h/$name.dtb" 2>/dev/null > "$WORK/h/$name.dts"; dtc -I dtb -O dts "$WORK/h/ref-$name.dtb" 2>/dev/null > "$WORK/h/ref-$name.dts"
      echo "  vs REF_SRC: $(diff "$WORK/h/ref-$name.dts" "$WORK/h/$name.dts" | grep -c '^[<>]') differing lines"
    fi
  done < <(scenarios)
  # rev 12's late /chosen m4,board-dtb-max edit on the final tree (host, real libfdt)
  gcc -O1 -w -I"$TREE/src/libfdt" -I"$TREE/src" -o "$WORK/chosen-test" $T/dt/chosen-test.c $TREE/src/standalone/fdt-check.c $TREE/src/libfdt/fdt.c $TREE/src/libfdt/fdt_ro.c $TREE/src/libfdt/fdt_rw.c $TREE/src/libfdt/fdt_wip.c $TREE/src/libfdt/fdt_strerror.c $TREE/src/libfdt/fdt_addresses.c $TREE/src/libfdt/fdt_empty_tree.c $TREE/src/libfdt/fdt_sw.c
  local ct; ct=$("$WORK/chosen-test" "$WORK/h/board.dtb" "$WORK/h/chosen.dtb"); echo "chosen-test: $ct"
  obs harness.chosen-test "$(echo "$ct" | sed 's/ /_/g')"
  # private hashes, when the private reference is at hand
  local privref=$PRIVATE/private-$(basename "$REFERENCE")
  if [ -f "$privref" ]; then
    if diff <(LC_ALL=C sort "$WORK/obs/private-harness.txt") <(grep -v '^#' "$privref" | LC_ALL=C sort) > "$WORK/private-harness.diff"; then echo "output trees: identical to the private reference"
    else echo "output trees differ from the private reference:"; cat "$WORK/private-harness.diff"; bad=1; fi
  else echo "note: no $privref: output-tree hashes not compared"; fi
  [ $bad = 0 ]
  echo "SUMMARY $(scenarios | wc -l | tr -d ' ') scenarios + chosen-test"
}

# ---- 7. pmp-test ---------------------------------------------------------------------------------------------
step_pmp() {
  need_built; need_private; need dtc; mkdir -p "$WORK/pmp" "$WORK/h"; local T=$M4_HERE/tests S=$TREE/src/standalone
  if [ ! -f "$S/pmp-publish.c" ]; then echo "no pmp-publish.c in this tree (before rev 13)"; return 77; fi
  python3 -I $T/dt/make-pmp-board.py "$BOARD" "$WORK/pmp/board-pmp.dtb" > /dev/null
  [ -x "$WORK/h/harness" ] || bash $T/dt/build-harness.sh "$TREE" "$WORK/h/harness" "$BOARD" "$WORK/h"
  "$WORK/h/harness" "$ADTREF" "$WORK/pmp/board-pmp.dtb" "$WORK/pmp/final-pmp.dtb" 2> "$WORK/pmp/harness-pmp.log"
  "$WORK/h/harness" "$ADTREF" "$BOARD" "$WORK/pmp/final-nopmp.dtb" 2> "$WORK/pmp/harness-nopmp.log"
  gcc -O1 -Wall -w -I"$S" -I"$TREE/src" -I"$TREE/src/libfdt" -o "$WORK/pmp/pmp-test" $T/pmp/pmp-test.c "$S/pmp-publish.c" $T/tools/adt-double.c \
      $TREE/src/libfdt/fdt.c $TREE/src/libfdt/fdt_ro.c $TREE/src/libfdt/fdt_rw.c $TREE/src/libfdt/fdt_wip.c $TREE/src/libfdt/fdt_strerror.c $TREE/src/libfdt/fdt_addresses.c $TREE/src/libfdt/fdt_empty_tree.c $TREE/src/libfdt/fdt_sw.c $S/fdt-check.c
  ( cd "$WORK/pmp" && ./pmp-test "$ADTREF" final-pmp.dtb final-nopmp.dtb ) | tee "$WORK/pmp/pmp-test.out"
  grep -q '^PASS (0 failures)' "$WORK/pmp/pmp-test.out"
  obs pmp.result PASS
  obs pmp.adt "$(sed -n 's/^ADT iop-pmp-nub: \(.*\)$/\1/p' "$WORK/pmp/pmp-test.out" | sed 's/ /_/g')"
  obs pmp.s1.128k "$(sed -n 's/^\[cap 128 KiB\] S1 \(.*\)$/\1/p' "$WORK/pmp/pmp-test.out" | sed 's/ /_/g')"
  obs pmp.s1.256k "$(sed -n 's/^\[cap 256 KiB\] S1 \(.*\)$/\1/p' "$WORK/pmp/pmp-test.out" | sed 's/ /_/g')"
  echo "SUMMARY pmp-test PASS, $(grep -c '' "$WORK/pmp/pmp-test.out") lines"
}

# ---- 8. rev 14: AOP DAPF, AOP segment IOVAs, Bluetooth admission, /chosen records ------------------------------
step_rev14() {
  need_built; need_private; need dtc fdtput; local T=$M4_HERE/tests
  if [ ! -f "$TREE/src/standalone/aop-dapf.c" ]; then echo "no aop-dapf.c in this tree (before rev 14)"; return 77; fi
  [ -x "$WORK/h/harness" ] || { mkdir -p "$WORK/h"; bash $T/dt/build-harness.sh "$TREE" "$WORK/h/harness" "$BOARD" "$WORK/h"; }
  mkdir -p "$WORK/rev14"
  bash $T/rev14/run-rev14-test.sh "$TREE" "$ADTREF" "$BOARD" "$WORK/h/harness" "$WORK/rev14" | tee "$WORK/rev14/run.out"
  sed -n 's/^obs //p' "$WORK/rev14/run.out" >> "$OBS"
  echo "SUMMARY dapf $(sed -n 's/^obs rev14.dapf-test //p' "$WORK/rev14/run.out"), timing $(sed -n 's/^obs rev14.timing-test //p' "$WORK/rev14/run.out"), port-1 admit $(sed -n 's/^obs rev14.t-port1.bt-admit //p' "$WORK/rev14/run.out")"
}

# ---- 9. the per-unit audit tools (m4/tools, HA) ---------------------------------------------------------------
step_dcpext() {
  need_built; need_private; need dtc
  if ! grep -q m4_dcpext_fixup "$TREE/src/standalone/dcp-fdt.c"; then echo "no m4_dcpext_fixup in this tree (before the dcpext regions)"; return 77; fi
  mkdir -p "$WORK/dcpext"
  bash $M4_HERE/tests/dcpext/run-dcpext-test.sh "$TREE" "$ADT" "$WORK/dcpext" | tee "$WORK/dcpext/run.out"
  tail -1 "$WORK/dcpext/run.out" | grep -q ' 0 failed'
}
step_audit() {
  need_built; need_private; local T=$M4_HERE/tools
  [ -f "$T/audit-tree.py" ] && [ -f "$T/gates-unit.py" ] && [ -f "$T/usb-contract-id.py" ] || { echo "m4/tools has no audit tools in this checkout"; return 77; }
  local n; n=$(python3 -I "$T/audit-tree.py" "$SRC" --adt "$ADT" --install "$PRIVATE/private-level1-install.h" 2>&1 | tee "$WORK/audit-tree.out" | sed -n 's/^audit-tree: \([0-9]*\) finding.*/\1/p'); obs audit.tree.findings "${n:-0}"
  obs audit.gates "$(python3 -I "$T/gates-unit.py" check "$ADT" "$TREE/src/standalone/gates.h" "$TREE/src/standalone/gates-unit.h" 2>&1 | tee "$WORK/gates-unit.out" | tail -1 | sed 's/ /_/g')"
  obs audit.usb-contract "$(python3 -I "$T/usb-contract-id.py" "$ADT" "$TREE/src/standalone/usb-contract-id.h" 2>&1 | tail -1 | sed 's/ /_/g')"
  echo "SUMMARY tree findings ${n:-0}; $(tail -1 "$WORK/gates-unit.out")"
}

if [ "${1:-}" = --list ]; then sed -n 's/^#   \([a-z0-9]*\) .*/\1/p' "$0" | head -12; exit 0; fi
WANT=${*:-all}
echo "m4 loader check: SRC=$SRC VERSION=$VERSION WORK=$WORK"
for s in build objects qemu e2e decision harness pmp rev14 dcpext audit; do step $s; done
{ echo "# observed by m4/check.sh ($(date -u +%Y-%m-%dT%H:%MZ)); copy to m4/reference/ to make it the reference of a new revision"; echo "version $VERSION"
  cat "$WORK"/obs/{build,objects,qemu,e2e,decision,harness,pmp,rev14,dcpext,audit}.txt 2>/dev/null; [ -f "$WORK/objects.txt" ] && sed 's/^\([0-9a-f]*\)  \(.*\)$/object \1 \2/' "$WORK/objects.txt"; } > "$WORK/observed.txt"
echo "----"
if [ $FAILS -gt 0 ]; then echo "CHECK FAILED: $PASSES passed, $FAILS failed, $SKIPS skipped (logs: $WORK/logs)"; exit 1
elif [ $SKIPS -gt 0 ]; then echo "CHECK PASSED with $SKIPS skipped: $PASSES passed (logs: $WORK/logs)"; exit 0
else echo "CHECK PASSED: all $PASSES steps (logs: $WORK/logs)"; exit 0; fi
