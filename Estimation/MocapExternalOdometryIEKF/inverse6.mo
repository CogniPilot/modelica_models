within Estimation.MocapExternalOdometryIEKF;
function inverse6 "Gauss-Jordan inverse for a 6x6 matrix"
  input Real matrix[6, 6];
  output Real matrix_inv[6, 6];
  output Real ok "1 when inversion succeeded, 0 when singular";
protected
  Real a[6, 6];
  Real diag;
  Real factor;
algorithm
  a := matrix;
  matrix_inv := identity(6);
  ok := 1.0;

  for pivot in 1:6 loop
    if ok > 0.5 then
      diag := a[pivot, pivot];
      if abs(diag) < 1.0e-18 then
        matrix_inv := identity(6);
        ok := 0.0;
      else
        for col in 1:6 loop
          a[pivot, col] := a[pivot, col] / diag;
          matrix_inv[pivot, col] := matrix_inv[pivot, col] / diag;
        end for;

        for row in 1:6 loop
          if row <> pivot then
            factor := a[row, pivot];
            for col in 1:6 loop
              a[row, col] := a[row, col] - factor * a[pivot, col];
              matrix_inv[row, col] :=
                matrix_inv[row, col] - factor * matrix_inv[pivot, col];
            end for;
          end if;
        end for;
      end if;
    end if;
  end for;
end inverse6;
