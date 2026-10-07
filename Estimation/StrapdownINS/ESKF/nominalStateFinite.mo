within Estimation.StrapdownINS.ESKF;

function nominalStateFinite
  "True when every published nominal state component is finite"
  input Real position[3];
  input Real velocity[3];
  input Real quaternion[4];
  output Boolean finite;
algorithm
  finite := true;
  for axis in 1:3 loop
    finite := finite
      and abs(position[axis]) < FiniteMagnitudeLimit
      and abs(velocity[axis]) < FiniteMagnitudeLimit;
  end for;
  for component in 1:4 loop
    finite := finite and abs(quaternion[component]) < FiniteMagnitudeLimit;
  end for;
end nominalStateFinite;
