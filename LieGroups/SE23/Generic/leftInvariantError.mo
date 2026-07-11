within LieGroups.SE23.Generic;
function leftInvariantError
  "Log(reference^(-1) actual), a local error coordinate for analysis"
  input Element reference;
  input Element actual;
  output Real tangent[9];
algorithm
  tangent := log_map(product(inverse(reference), actual));
end leftInvariantError;
