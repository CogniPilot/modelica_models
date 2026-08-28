within Tests.HorizonChecks;

function incrementalResidual
  "Worst |incremental predictor - one fold and compose| with no correction"
  input Integer count(min = 1);
  input Real dt(unit = "s");
  input Real gravityWorldEnu_m_s2[3];
  output Real worst[3] "position [m], velocity [m/s], attitude [rad]";
protected
  Real correction[9];
algorithm
  // The predictor's cheap path composes only the newest delta onto its own
  // previous answer; its expensive path recomposes the whole buffer. With no
  // correction the two must be the same element, which is what lets the block
  // take the cheap path on every tick that did not fuse anything. This is
  // rebaseResidual with a zero shift, so the two paths are exercised by one
  // driver and cannot drift apart.
  correction := zeros(9);
  worst := Tests.HorizonChecks.rebaseResidual(
    count, dt, gravityWorldEnu_m_s2, correction);
end incrementalResidual;
