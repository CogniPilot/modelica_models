within Optimization.Internal;
function projectCone "Euclidean projection onto the cone ||t|| <= mu n"
  input Real normal;
  input Real tangent1;
  input Real tangent2;
  input Real mu(min = 0.0) "Friction coefficient";
  output Real projectedNormal;
  output Real projectedTangent1;
  output Real projectedTangent2;
protected
  Real slip;
  Real scale;
algorithm
  slip := sqrt(tangent1 * tangent1 + tangent2 * tangent2);
  // The normal >= 0 test matters for mu = 0 with zero slip, where slip <= mu * normal also holds
  // for a negative normal; such a point must fall through to the polar case and project to zero.
  if normal >= 0.0 and slip <= mu * normal then
    projectedNormal := normal;
    projectedTangent1 := tangent1;
    projectedTangent2 := tangent2;
  elseif mu * slip <= -normal then
    projectedNormal := 0.0;
    projectedTangent1 := 0.0;
    projectedTangent2 := 0.0;
  else
    scale := (normal + mu * slip) / (1.0 + mu * mu);
    projectedNormal := scale;
    projectedTangent1 := mu * scale * tangent1 / slip;
    projectedTangent2 := mu * scale * tangent2 / slip;
  end if;
end projectCone;
