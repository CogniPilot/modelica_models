within SLAM.Inertial;
model SLAMRotationLog
  import RGBDProperRotation = SLAM.Localization.RGBDProperRotation;

  constant Integer spaceDimension = 3;
  constant Integer quaternionDimension = 4;
  input Real rotation[spaceDimension,spaceDimension] = identity(spaceDimension);
  output Real vector[spaceDimension];
  output Real angle;
  output Real valid;
protected
  RGBDProperRotation rotationCheck(rotation=rotation);
  Real candidate[quaternionDimension];
  Real raw[quaternionDimension];
  Real quaternion[quaternionDimension];
  Real norm;
  Real sine;
  Real scale;
equation
  candidate = {2.0*sqrt(max(0.0,1.0+rotation[1,1]+rotation[2,2]+rotation[3,3])),
    2.0*sqrt(max(0.0,1.0+rotation[1,1]-rotation[2,2]-rotation[3,3])),
    2.0*sqrt(max(0.0,1.0-rotation[1,1]+rotation[2,2]-rotation[3,3])),
    2.0*sqrt(max(0.0,1.0-rotation[1,1]-rotation[2,2]+rotation[3,3]))};
  raw = if noEvent(rotation[1,1]+rotation[2,2]+rotation[3,3] > 0.0) then
      {candidate[1]/4.0,(rotation[3,2]-rotation[2,3])/max(candidate[1],1e-12),
       (rotation[1,3]-rotation[3,1])/max(candidate[1],1e-12),
       (rotation[2,1]-rotation[1,2])/max(candidate[1],1e-12)}
    elseif noEvent(rotation[1,1] > rotation[2,2] and rotation[1,1] > rotation[3,3]) then
      {(rotation[3,2]-rotation[2,3])/max(candidate[2],1e-12),candidate[2]/4.0,
       (rotation[1,2]+rotation[2,1])/max(candidate[2],1e-12),
       (rotation[1,3]+rotation[3,1])/max(candidate[2],1e-12)}
    elseif noEvent(rotation[2,2] > rotation[3,3]) then
      {(rotation[1,3]-rotation[3,1])/max(candidate[3],1e-12),
       (rotation[1,2]+rotation[2,1])/max(candidate[3],1e-12),candidate[3]/4.0,
       (rotation[2,3]+rotation[3,2])/max(candidate[3],1e-12)}
    else {(rotation[2,1]-rotation[1,2])/max(candidate[4],1e-12),
       (rotation[1,3]+rotation[3,1])/max(candidate[4],1e-12),
       (rotation[2,3]+rotation[3,2])/max(candidate[4],1e-12),candidate[4]/4.0};
  norm = sqrt(sum(raw.^2));
  quaternion = (if noEvent(raw[1] < 0.0) then -1.0 else 1.0)*raw/max(norm,1e-12);
  sine = sqrt(sum(quaternion[i]^2 for i in 2:quaternionDimension));
  angle = 2.0*atan2(sine,quaternion[1]);
  scale = if noEvent(sine < 1e-8) then 2.0 else angle/max(sine,1e-12);
  valid = rotationCheck.valid;
  for i in 1:spaceDimension loop
    vector[i] = if noEvent(valid > 0.5) then scale*quaternion[i+1] else 0.0;
  end for;
end SLAMRotationLog;
