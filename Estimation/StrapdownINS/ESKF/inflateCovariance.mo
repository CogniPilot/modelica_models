within Estimation.StrapdownINS.ESKF;

function inflateCovariance
  "Ramp position and velocity covariance toward the mission envelope"
  input Covariance covariance;
  input VarianceLimits limits;
  input Real dt(unit = "s");
  input Real timeConstant_s
    "e-folding time of the variance ramp; must be strictly positive";
  output Covariance inflated;
protected
  Real bound[TangentLength];
  Real diagonalAddend[TangentLength];
  Real stepFactor;
algorithm
  bound := cat(1,
    limits.position_m2,
    limits.velocity_m2_s2,
    limits.attitude_rad2,
    limits.gyroscopeBias_rad2_s2,
    limits.accelerometerBias_m2_s4);

  stepFactor := exp(dt / timeConstant_s);

  for row in 1:TangentLength loop
    // Affirmative guard: grow only a diagonal entry that is positive AND
    // still below the bound. A zero, negative or NaN variance yields a
    // zero addend rather than an infinite or NaN one, leaving the
    // degenerate matrix for the Cholesky guard in solveSPD to report.
    //
    // The ADDEND for this entry: what one ramp step would have added
    // multiplicatively, capped so the entry never passes the envelope.
    diagonalAddend[row] := if row <= 6 and covariance[row, row] > 0.0
        and covariance[row, row] < bound[row]
      then min((stepFactor * stepFactor - 1.0) * covariance[row, row],
               bound[row] - covariance[row, row])
      else 0.0;
  end for;

  inflated := covariance;
  for row in 1:TangentLength loop
    inflated[row, row] := covariance[row, row] + diagonalAddend[row];
  end for;
end inflateCovariance;
