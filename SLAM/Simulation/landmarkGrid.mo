within SLAM.Simulation;

function landmarkGrid "Fixed planar landmarks with stable row-major identities"
  input Integer rows(min=1);
  input Integer columns(min=1);
  input Real spacing_m(min=0.0);
  input Real centerWorld_m[3] = zeros(3);
  output Real landmarksWorld_m[rows * columns, 3];
protected
  Integer row;
  Integer column;
algorithm
  row := 1;
  column := 1;
  for landmark in 1:rows * columns loop
    landmarksWorld_m[landmark, :] := centerWorld_m
      + {(column - 0.5 * (columns + 1)) * spacing_m,
         (row - 0.5 * (rows + 1)) * spacing_m, 0.0};
    column := column + 1;
    if column > columns then
      column := 1;
      row := row + 1;
    end if;
  end for;
end landmarkGrid;
