#!/usr/bin/env bash
# Randomized rumoca-binary check of the synthesized barometer H.
#
# The rumoca Solve IR evaluates the flattened barometer (EskfJacobianProofFlat)
# directly. For each state in states.txt this computes the third row of R(qBar),
# substitutes it and the position, runs the model under the rumoca binary, and
# reads max|Hsynth - Hhand| from the report. Hhand here is R[3,:], so a zero gap
# is the rumoca binary confirming the synthesized H equals the world-up body axis.
#
# Overridable: RUMOCA.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUMOCA="${RUMOCA:-rumoca}"
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
MAX=0; k=0
while read -r qw qx qy qz px py pz vx vy vz; do
  [ -z "${qw:-}" ] && continue
  k=$((k+1))
  read r31 r32 r33 < <(awk -v qw=$qw -v qx=$qx -v qy=$qy -v qz=$qz 'BEGIN{
    n=sqrt(qw*qw+qx*qx+qy*qy+qz*qz); a=qw/n;b=qx/n;c=qy/n;d=qz/n;
    printf("%.15g %.15g %.15g\n",2*(b*d-a*c),2*(c*d+a*b),a*a-b*b-c*c+d*d)}')
  D="$WORK/s$k"; mkdir -p "$D"
  sed -e "s/parameter Real pBar\[3\] = {[^}]*}/parameter Real pBar[3] = {$px, $py, $pz}/" \
      -e "s/parameter Real Rrow\[3\] = {[^}]*}/parameter Real Rrow[3] = {$r31, $r32, $r33}/" \
      "$HERE/EskfJacobianProofFlat.mo" > "$D/EskfJacobianProofFlat.mo"
  ( cd "$D" && "$RUMOCA" sim EskfJacobianProofFlat.mo --model EskfJacobianProofFlat.BaroFlat --t-end 0.0 -o f.html >/dev/null 2>&1 )
  vals=$(grep -o '"allData":\[\[.*\]\]' "$D/f.html" | grep -o '\[[-0-9.eE]*\]' | tr -d '[]')
  mdh=$(echo "$vals" | tail -3 | head -1)
  printf "state %2d  R[3,:]=[% .4f % .4f % .4f]  max|Hsynth-Hhand|=%s\n" "$k" "$r31" "$r32" "$r33" "$mdh"
  MAX=$(awk -v a=$MAX -v b=$mdh 'BEGIN{b=(b<0)?-b:b; print (b>a)?b:a}')
done < "$HERE/states.txt"
echo "===== rumoca Solve IR: max|Hsynth-Hhand| over $k states = $MAX ====="
