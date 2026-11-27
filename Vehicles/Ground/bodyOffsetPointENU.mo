within Vehicles.Ground;
function bodyOffsetPointENU "Transform an offset from a rigid body origin into world ENU"
  input Real originENU[3](each unit="m");
  input Real attitude[3](each unit="rad") "Heading, pitch up, roll right down";
  input Real offset[3](each unit="m") "Right, forward and up offsets from the body origin";
  output Real pointENU[3](each unit="m");
protected
  Real right, up, forward;
algorithm
  right := offset[1]*cos(attitude[3])+offset[3]*sin(attitude[3]);
  up := offset[3]*cos(attitude[3])-offset[1]*sin(attitude[3]);
  forward := offset[2]*cos(attitude[2])-up*sin(attitude[2]);
  pointENU := originENU + {forward*sin(attitude[1])+right*cos(attitude[1]),
    forward*cos(attitude[1])-right*sin(attitude[1]),up*cos(attitude[2])+offset[2]*sin(attitude[2])};
end bodyOffsetPointENU;
