within Vehicles.Rdd2;

block NavigationEstimator
  "RDD2 eFMU implementation of the stable navigation estimator boundary"
  extends Estimation.MixedInvariantNavigationEstimator;

  annotation(Documentation(info = "<html>
    <p>Export this block as the default RDD2 estimator eFMU. Alternate filter
    blocks extend <code>Avionics.PartialNavigationEstimator</code>
    and therefore replace this artifact without changing any sensor input,
    navigation output, status field, or controller wiring.</p>
  </html>"));
end NavigationEstimator;
