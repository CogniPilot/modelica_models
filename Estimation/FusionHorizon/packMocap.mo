within Estimation.FusionHorizon;

function packMocap "Flatten one mocap sample into a queue row"
  input Avionics.MocapSample measurement;
  output Real row[MocapMeasurementLength];
algorithm
  // THE ONE PLACE THE MOCAP QUEUE LAYOUT IS KNOWN, together with
  // unpackMocap. The timestamp is column 1 in every source's layout and that
  // is not a convention, it is the contract stepQueue is written against: the
  // queue kernel orders and ripens rows by that column and knows nothing else
  // about any of them.
  //
  // valid and fresh are not carried. A row is in the queue only because it
  // arrived valid, and both flags are asserted on the tick it is delivered,
  // so storing them would store a constant.
  row := cat(1,
    {measurement.timestamp_s},
    measurement.positionWorldEnu_m,
    measurement.quaternionWorldBody,
    measurement.positionCovarianceWorld_m2[1, :],
    measurement.positionCovarianceWorld_m2[2, :],
    measurement.positionCovarianceWorld_m2[3, :],
    measurement.attitudeCovarianceBody_rad2[1, :],
    measurement.attitudeCovarianceBody_rad2[2, :],
    measurement.attitudeCovarianceBody_rad2[3, :]);
end packMocap;
