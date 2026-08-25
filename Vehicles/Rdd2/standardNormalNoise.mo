within Vehicles.Rdd2;

function standardNormalNoise
  "Reproducible standard-normal sample from an indexed sine hash"
  input Real sampleIndex;
  input Real channel;
  input Real seed;
  output Real sample;
protected
  constant Real pi = 3.1415926535897932384626433832795;
  Real phase1;
  Real phase2;
  Real uniform1;
  Real uniform2;
algorithm
  phase1 := (sampleIndex + 1) * (sampleIndex + 1)
      * (12.9898 + 0.731 * channel)
    + (sampleIndex + 1) * (0.113 + 0.017 * channel)
    + seed * 0.0174532925199433;
  phase2 := (sampleIndex + 1) * (sampleIndex + 1)
      * (78.233 + 0.527 * channel)
    + (sampleIndex + 1) * (0.193 + 0.029 * channel)
    + seed * 0.01113552872566;
  // asin(sin(phase)) is a constant-slope triangle wave. For an
  // irrational quadratic phase it maps the indexed phase uniformly onto
  // [-pi/2, pi/2] without mod(), floor(), or another runtime quotient
  // discontinuity. The changing quadratic phase step avoids the strong
  // lag-one correlation of a linear triangle-wave sequence.
  uniform1 := min(max(
    0.5 + asin(sin(phase1)) / pi, 1.0e-12), 1.0 - 1.0e-12);
  uniform2 := min(max(
    0.5 + asin(sin(phase2)) / pi, 1.0e-12), 1.0 - 1.0e-12);
  sample := sqrt(-2.0 * log(uniform1)) * cos(2.0 * pi * uniform2);
end standardNormalNoise;
