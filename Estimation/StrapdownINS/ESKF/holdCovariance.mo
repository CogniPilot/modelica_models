within Estimation.StrapdownINS.ESKF;

function holdCovariance
  "Grow covariance by one interval of process noise with the state held"
  input Estimation.StrapdownINS.ESKF.Covariance covariance;
  input Real dt(unit = "s");
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output Estimation.StrapdownINS.ESKF.Covariance grown;
protected
  Real A[TangentLength, TangentLength];
  Real G[TangentLength, ProcessNoiseLength];
  Real transition[TangentLength, TangentLength];
  Estimation.StrapdownINS.ProcessNoiseCovariance continuousNoise;
  Estimation.StrapdownINS.ESKF.Covariance discreteNoise;
algorithm
  // The covariance step for a tick with NO usable IMU sample. The nominal
  // state is held by the caller; here the uncertainty about that held
  // state grows by the process noise accumulated over the interval.
  //
  // The error transition IS applied, evaluated at zero corrected angular
  // velocity and zero corrected specific force.
  //
  // An earlier version added only the process-noise integral, reasoning
  // that without an IMU sample there is no operating point to linearize
  // about. That was wrong in the part that matters most. The dominant
  // term in position uncertainty over a blind interval is not process
  // noise at all, it is the velocity uncertainty the vehicle is flying
  // on: dp/dv = I is pure kinematics and holds whatever the IMU is
  // doing, so omitting the transition omitted the P_vv * dt^2 growth and
  // left only the O(dt^3) accelerometer term. At dt = 1 ms that term is
  // about 3e-13 against a position variance near 0.019 -- below one ulp
  // of binary32, so the position block did not grow AT ALL in the
  // generated flight code. Measured over a 10 s blind window: position
  // variance change exactly 0.0. The function documented the growth it
  // was not performing.
  //
  // The blocks the zero-input transition does invent are the ones
  // coupling attitude to specific force, and those are multiplied by a
  // zero specific force, so nothing is fabricated there either. The
  // result is symmetric positive semi-definite as F*P*F' (a congruence
  // of a PSD matrix) plus a PSD noise integral.
  //
  // Cost is bounded by, and strictly below, the full predict() this
  // replaces on the same branch -- the nominal propagation and the
  // reset Jacobian are absent -- so the block's worst-case execution
  // time is still set by the IMU-valid path and does not grow.
  // limitCovariance caps the result at the mission envelope on the
  // caller's next line, so an unbounded IMU dropout saturates at the
  // envelope rather than growing without bound.
  A := continuousTransition(zeros(3), zeros(3));
  G := noiseInputMatrix();
  transition := discreteTransition(A, dt);
  continuousNoise := processNoiseMatrix(processNoise);
  discreteNoise := discreteProcessCovariance(A, G, continuousNoise, dt);
  grown := LinearAlgebra.symmetrize(
    transition * covariance * transpose(transition) + discreteNoise);
end holdCovariance;
