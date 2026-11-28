within Marine.Wreck;
function steepTiltMoment "Body-axis attitude moment using gravity as the tilt reference"
  input Real up[3] "Estimated world up, unit vector in body coordinates";
  input Real bank(unit="rad") "Commanded FLU roll";
  input Real pitch(unit="rad") "Commanded FLU pitch, positive nose down";
  input Real gyro[3](each unit="rad/s") "Body angular velocity";
  input Real yawMoment(unit="N.m") "Moment about world up";
  output Real moment[3](each unit="N.m") "Commanded moment in body axes";
protected
  Real desiredUp[3],tilt[3],transverseRate[3];
algorithm
  desiredUp:={-sin(pitch),sin(bank)*cos(pitch),cos(bank)*cos(pitch)};
  tilt:=cross(desiredUp,up);
  transverseRate:=gyro-up*(up*gyro);
  moment:=10*tilt-4*transverseRate+up*yawMoment;
end steepTiltMoment;
