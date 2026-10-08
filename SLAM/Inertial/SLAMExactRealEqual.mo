within SLAM.Inertial;
// MLS 3.5 permits exact Real equality inside functions. This helper preserves
// IEEE equality: signed zeros compare equal; NaN compares unequal to every value.
function SLAMExactRealEqual
  input Real left;
  input Real right;
  output Boolean equal;
algorithm
  equal := left == right;
end SLAMExactRealEqual;
