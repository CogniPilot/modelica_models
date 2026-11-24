within Vision.Sensors;
package MicFusion9000 "Bosch MIC-9502-Z30WVF dual-imager simulation"
  constant Real rad = 2*asin(1.0)/180 "Radians per degree";
  constant Integer visibleSize[2] = {1920,1080} "Full HD stream, width then height";
  constant Integer thermalSize[2] = {640,480};
  constant Real visibleFocalRange[2](each unit="m") = {0.0043,0.129};
  constant Real visibleHorizontalFov[2](each unit="rad") = {63.7,2.3}*rad "Wide, tele";
  constant Real thermalFov[2](each unit="rad") = {12.4,9.3}*rad "Horizontal, vertical";
  constant Real thermalFocalLength(unit="m") = 0.050;
  constant Real thermalPixelPitch(unit="m") = 17e-6;
  constant Real thermalBand[2](each unit="m") = {8e-6,14e-6};
  constant Real thermalNetdLimit(unit="K") = 0.072 "Upper bound at room temperature with noise reduction enabled";
  constant Real thermalFrameRate(unit="Hz") = 30;
  constant Real thermalNearFocus(unit="m") = 84 "Lower end of specified in-focus range";
  constant Real visibleMaxFrameRate(unit="Hz") = 60;
  constant Real panRateLimit(unit="rad/s") = 120*rad;
  constant Real tiltRateLimit(unit="rad/s") = 90*rad;
  constant Real tiltLimits[2](each unit="rad") = {-56,90}*rad "Upright installation";

  function visibleIntrinsics "Centered pinhole intrinsics for full-frame 16:9 visible output"
    input Real opticalZoom "Focal length divided by its wide-end value";
    input Real width(unit="1") = visibleSize[1];
    input Real height(unit="1") = visibleSize[2];
    output Real k[4] "fx, fy, cx, cy, in pixels from the top-left image edge";
  protected
    Real fraction;
    Real normalizedFocal;
  algorithm
    assert(width > 0 and height > 0, "Positive image dimensions required");
    fraction := (min(30,max(1,opticalZoom))-1)/29;
    normalizedFocal := (1-fraction)/tan(visibleHorizontalFov[1]/2)
                       + fraction/tan(visibleHorizontalFov[2]/2);
    k := {width*normalizedFocal/2,height*(16.0/9)*normalizedFocal/2,width/2,height/2};
  end visibleIntrinsics;

  function thermalIntrinsics "Fixed-focus thermal channel at its native resolution"
    output Real k[4] "fx, fy, cx, cy in pixels from the top-left image edge";
  algorithm
    k := {thermalSize[1]/(2*tan(thermalFov[1]/2)),thermalSize[2]/(2*tan(thermalFov[2]/2)),
          thermalSize[1]/2,thermalSize[2]/2};
  end thermalIntrinsics;

  block UprightMount "Upright mount driven by external angular-rate requests"
    input Real requestedPanRate(unit="rad/s") "Positive about upright axis";
    input Real requestedTiltRate(unit="rad/s") "Positive toward zenith";
    parameter Real initialPan(unit="rad") = 0;
    parameter Real initialTilt(unit="rad", min=tiltLimits[1], max=tiltLimits[2]) = 0;
    output Real pan(unit="rad",start=initialPan,fixed=true) "Unwrapped continuous pan";
    output Real tilt(unit="rad",start=initialTilt,fixed=true);
  protected
    Real rate(unit="rad/s");
  equation
    der(pan) = min(panRateLimit,max(-panRateLimit,requestedPanRate));
    rate = min(tiltRateLimit,max(-tiltRateLimit,requestedTiltRate));
    der(tilt) = if (tilt >= tiltLimits[2] and rate > 0) or
                   (tilt <= tiltLimits[1] and rate < 0) then 0 else rate;
  end UprightMount;

  function bandRadiance "Ideal uniform-response 8 to 14 micrometre blackbody band radiance"
    input Real temperature(unit="K");
    output Real radiance(unit="W/(m2.sr)");
  protected
    constant Real h = 6.62607015e-34;
    constant Real c = 299792458;
    constant Real kb = 1.380649e-23;
    constant Integer n = 24;
    constant Real step = (thermalBand[2]-thermalBand[1])/n;
    Real wavelength;
  algorithm
    assert(temperature > 0, "Temperature must be in kelvin");
    radiance := 0;
    for i in 1:n loop
      wavelength := thermalBand[1]+(i-0.5)*step;
      radiance := radiance + step*2*h*c^2/(wavelength^5*(exp(h*c/(wavelength*kb*temperature))-1));
    end for;
  end bandRadiance;

  function windowView "Wet window at one exposure, indexed by rain episode time"
    input Real elapsed(unit="s") "Time since start of rain";
    input Real rainDuration(unit="s") "Use elapsed for ongoing rain; hold duration after rain stops";
    input Integer seed = 1 "Station-specific repeatable pattern";
    input Boolean thermal = false "Select the adjacent thermal window";
    input Real wipeInterval(unit="s") = 8 "Time between automatic wipe starts";
    input Real sweepTime(unit="s") = 1.2 "Out and back";
    input Real dryTime(unit="s") = 11 "Individual droplet lifetime";
    output Real drops[8,3] "Normalized local window x, y (down), radius as fraction of window width; zero radius is dry";
    output Real blade[4] "Blade endpoints in the same local window coordinates; parked outside both images";
  protected
    Real t;
    Real endRain;
    Real pane;
    Real phase;
    Real angle;
    Real arrival;
    Real interval;
    Real offset;
    Real generation;
    Real u;
    Real v;
    Real a;
    Real passage;
    Real lastPass;
    Real firstPass;
    Real returnPass;
    Real lastCycle;
    Real sweepClock;
  algorithm
    assert(wipeInterval >= sweepTime and sweepTime > 0 and dryTime > 0, "Invalid window timing");
    t := max(0,elapsed);
    endRain := min(t,max(0,rainDuration));
    pane := if thermal then 1 else 0;
    // Finish an underway wipe after rainfall ceases. The first wipe starts one interval in.
    sweepClock := min(t,floor(endRain/wipeInterval)*wipeInterval+sweepTime);
    phase := if sweepClock < wipeInterval then 0 else min(1,mod(sweepClock,wipeInterval)/sweepTime);
    angle := -1.36+2.72*(1-abs(2*phase-1));
    blade := {1.5-pane*2,-0.45,1.5-pane*2+2.7*sin(angle),-0.45+2.7*cos(angle)};
    for i in 1:8 loop
      interval := 6+i*0.73;
      offset := 0.15+0.91*i+mod(0.29*seed+0.47*pane,1.7);
      generation := floor((endRain-offset)/interval);
      arrival := offset+generation*interval;
      u := mod(sin((seed*5.3+i*17.1+generation*71.3+pane)*1.91)*43758.5453,1);
      v := mod(sin((seed*7.9+i*11.3+generation*47.1+pane)*2.39)*32719.1231,1);
      a := atan2(u-(1.5-pane*2),v+0.45);
      passage := (a+1.36)/2.72*sweepTime/2;
      lastCycle := floor(sweepClock/wipeInterval)*wipeInterval;
      firstPass := lastCycle+passage;
      returnPass := lastCycle+sweepTime-passage;
      lastPass := if sweepClock >= returnPass then returnPass elseif sweepClock >= firstPass then firstPass
                  else lastCycle-wipeInterval+sweepTime-passage;
      drops[i,:] := {u,v,if generation >= 0 and t-arrival < dryTime and
        (lastPass < wipeInterval or lastPass < arrival) then (0.012+0.002*i)*(1-(t-arrival)/dryTime) else 0};
    end for;
  end windowView;

  annotation(Documentation(info="<html><p>Profile for the white-housing VF model in the Bosch MIC IP fusion 9000i data sheet (technical and ordering tables). The visible lens interpolates normalized focal length between published wide and tele fields of view. Thermal focus and optical field of view are fixed. Digital cropping, exposure control, optical distortion, boresight and timing offsets require separate calibration.</p><p>The mount models rate limits and upright end stops. It assumes instantaneous rate response. The radiance function predicts a blackbody reference signal using a flat spectral response. NETD describes differential sensitivity. Absolute temperature accuracy, detector counts, internal processing and object temperatures require independent measurements.</p><p>The integrated silicone blade clears glass and germanium windows. Its path, droplet distribution, drying and timing in windowView are illustrative configurable simulation assumptions. Each rain episode has its own clock. A common blade crosses the two local images at different times.</p></html>"));
end MicFusion9000;
