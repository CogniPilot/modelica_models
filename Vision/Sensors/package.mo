within Vision;

package Sensors
  "Camera profiles, calibration and image input contracts"
  annotation(Documentation(info = "<html>
    <h4>Camera stream profiles</h4>
<p><a href=\"modelica://Vision.Sensors.D435ImageProfile\">D435ImageProfile</a>
provides array dimensions and the default depth scale for a D435-type stream.
It is configuration data, not a hardware driver or a calibration estimator.</p>
<p>Use measured intrinsics and extrinsics for the actual device and stream
mode. RGB and depth may have different calibrations and acquisition timing.
Convert depth samples to metres once; an SDK-scale integer depth image is not
already a metric array. Use
<a href=\"modelica://SLAM.Simulation.LandmarkCamera\">LandmarkCamera</a> for
landmark-level synthetic observations without image processing.</p>
    </html>"));
end Sensors;
