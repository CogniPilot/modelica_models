within LieGroups.SE3.Generic;
function rightInvariantError
  "Log(actual reference^(-1)), a spatial local error coordinate"
  input Element reference;
  input Element actual;
  output Real tangent[6];
algorithm
  tangent := log_map(product(actual, inverse(reference)));
end rightInvariantError;
