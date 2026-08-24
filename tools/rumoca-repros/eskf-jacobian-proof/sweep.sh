#!/usr/bin/env bash
# Randomized measurement-Jacobian parity sweep.
#
# For each state in states.txt and each correction (Barometer, GpsPosition,
# MagnetometerYaw): substitute the nominal state into EskfJacobianProof.mo,
# expand the jacobian construct to portable Modelica with rumoca, evaluate it
# under OpenModelica, and read the maximum absolute element gaps
#   max|Hsynth - Hhand|  (synthesized H vs the shipped-library hand H)
#   max|Hsynth - Jfd|    (synthesized H vs central differences of h through retract)
# then report the maximum of each over all states.
#
# Overridable: RUMOCA, OMC, REPO_ROOT.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$HERE/../../.." && pwd)}"
RUMOCA="${RUMOCA:-rumoca}"
OMC="${OMC:-omc}"
SRC="$HERE/EskfJacobianProof.mo"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
declare -A MAXH MAXF
for M in Barometer GpsPosition MagnetometerYaw; do MAXH[$M]=0; MAXF[$M]=0; done
k=0
while read -r qw qx qy qz px py pz vx vy vz; do
  [ -z "${qw:-}" ] && continue
  k=$((k+1)); D="$WORK/s$k"; mkdir -p "$D"
  sed -e "s/parameter Real qRaw\[4\] = {[^}]*}/parameter Real qRaw[4] = {$qw, $qx, $qy, $qz}/" \
      -e "s/parameter Real pBar\[3\] = {[^}]*}/parameter Real pBar[3] = {$px, $py, $pz}/" \
      -e "s/parameter Real vBar\[3\] = {[^}]*}/parameter Real vBar[3] = {$vx, $vy, $vz}/" \
      "$SRC" > "$D/EskfJacobianProof.mo"
  for M in Barometer GpsPosition MagnetometerYaw; do
    MD="$D/$M"; mkdir -p "$MD"
    "$RUMOCA" compile "$D/EskfJacobianProof.mo" --model "EskfJacobianProof.$M" \
       --source-root "$REPO_ROOT" --emit-standard-modelica -o "$MD/Std.mo" >/dev/null 2>&1
    [ -s "$MD/Std.mo" ] || { echo "state $k $M: EMIT FAILED"; continue; }
    cat > "$MD/run.mos" <<EOF
setModelicaPath("$REPO_ROOT"); getErrorString();
loadModel(LieGroups); getErrorString();
loadFile("Std.mo"); getErrorString();
simulate(EskfJacobianProof.$M, stopTime=1.0, numberOfIntervals=2); getErrorString();
print("RESH " + String(val(maxDiffHand,0.0)) + "\n");
print("RESF " + String(val(maxDiffFd,0.0)) + "\n");
print("RESS " + String(val(sizeH,0.0)) + "\n");
EOF
    ( cd "$MD" && "$OMC" run.mos > omc.out 2>&1 )
    h=$(grep '^RESH ' "$MD/omc.out" | awk '{print $2}')
    f=$(grep '^RESF ' "$MD/omc.out" | awk '{print $2}')
    s=$(grep '^RESS ' "$MD/omc.out" | awk '{print $2}')
    [ -z "$h" ] && { echo "state $k $M: OMC FAILED"; continue; }
    printf "state %2d %-15s max|Hsynth-Hhand|=%.3e  max|Hsynth-Jfd|=%.3e  sizeH=%.4g\n" "$k" "$M" "$h" "$f" "$s"
    MAXH[$M]=$(awk -v a="${MAXH[$M]}" -v b="$h" 'BEGIN{print (b>a)?b:a}')
    MAXF[$M]=$(awk -v a="${MAXF[$M]}" -v b="$f" 'BEGIN{print (b>a)?b:a}')
  done
done < "$HERE/states.txt"
echo "================= AGGREGATE over $k states ================="
for M in Barometer GpsPosition MagnetometerYaw; do
  printf "%-15s  max|Hsynth-Hhand|=%.3e   max|Hsynth-Jfd|=%.3e\n" "$M" "${MAXH[$M]}" "${MAXF[$M]}"
done
