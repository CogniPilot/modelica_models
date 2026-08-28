within Tests;
package HorizonChecks
  "Residual drivers for the delayed fusion horizon and its output predictor"
  annotation(Documentation(info="<html>
    <p>Each driver runs one identity of
    <code>Estimation.FusionHorizon</code> two ways and returns the worst
    absolute disagreement, so the test model asserts on a scalar whose
    tolerance can be stated and whose scaling with the sample period can be
    checked. The identities are the ones the architecture rests on: that
    composing buffered deltas reproduces a single integration pass, that a
    corrected horizon pose reapplies the same buffered factors, and that the
    incremental predictor and a full recomposition are the same element.</p>
    <p>The synthetic stream is deliberately coning-rich and sculling-rich: a
    rotating angular-velocity vector with a steady yaw component and specific
    force oscillating on three incommensurate frequencies. A stream that is
    smooth in the body frame would let a wrong composition pass.</p>
  </html>"));
end HorizonChecks;
