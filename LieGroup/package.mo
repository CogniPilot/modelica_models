package LieGroup
  package SO3
    function skew
      input Real v[3] "Vector";
      output Real S[3, 3] "Skew-symmetric matrix such that S * x = cross(v, x)";

    algorithm
      S[1, 1] := 0;
      S[1, 2] := -v[3];
      S[1, 3] := v[2];
      S[2, 1] := v[3];
      S[2, 2] := 0;
      S[2, 3] := -v[1];
      S[3, 1] := -v[2];
      S[3, 2] := v[1];
      S[3, 3] := 0;
    end skew;

    function quaternionProduct
      input Real a[4] "Left quaternion w,x,y,z";
      input Real b[4] "Right quaternion w,x,y,z";
      output Real q[4] "Product a * b";

    algorithm
      q[1] := a[1] * b[1] - a[2] * b[2] - a[3] * b[3] - a[4] * b[4];
      q[2] := a[1] * b[2] + a[2] * b[1] + a[3] * b[4] - a[4] * b[3];
      q[3] := a[1] * b[3] - a[2] * b[4] + a[3] * b[1] + a[4] * b[2];
      q[4] := a[1] * b[4] + a[2] * b[3] - a[3] * b[2] + a[4] * b[1];
    end quaternionProduct;

    function quaternionConjugate
      input Real q[4] "Quaternion w,x,y,z";
      output Real q_conj[4] "Quaternion conjugate";

    algorithm
      q_conj[1] := q[1];
      q_conj[2] := -q[2];
      q_conj[3] := -q[3];
      q_conj[4] := -q[4];
    end quaternionConjugate;

    function quaternionNormalize
      input Real q[4] "Quaternion w,x,y,z";
      input Real eps = 1.0e-12 "Minimum norm";
      output Real q_unit[4] "Unit quaternion w,x,y,z";
    protected
      Real n;

    algorithm
      n := sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4]);
      if n > eps then
        q_unit[1] := q[1] / n;
        q_unit[2] := q[2] / n;
        q_unit[3] := q[3] / n;
        q_unit[4] := q[4] / n;
      else
        q_unit[1] := 1;
        q_unit[2] := 0;
        q_unit[3] := 0;
        q_unit[4] := 0;
      end if;
    end quaternionNormalize;

    function quaternionExp
      input Real phi[3] "Rotation vector in so(3)";
      output Real q[4] "Quaternion exp(phi) as w,x,y,z";
    protected
      Real theta2;
      Real theta;
      Real half_theta;
      Real scale;

    algorithm
      theta2 := phi[1] * phi[1] + phi[2] * phi[2] + phi[3] * phi[3];
      theta := sqrt(theta2);
      half_theta := 0.5 * theta;
      scale := if theta > 1.0e-8 then sin(half_theta) / theta else 0.5 - theta2 / 48.0;
      q[1] := cos(half_theta);
      q[2] := scale * phi[1];
      q[3] := scale * phi[2];
      q[4] := scale * phi[3];
    end quaternionExp;

    function quaternionLog
      input Real q[4] "Quaternion w,x,y,z";
      output Real phi[3] "Shortest-path rotation vector in so(3)";
    protected
      Real q_unit[4];
      Real v_norm;
      Real angle;
      Real scale;

    algorithm
      q_unit := quaternionNormalize(q);
      if q_unit[1] < 0 then
        q_unit[1] := -q_unit[1];
        q_unit[2] := -q_unit[2];
        q_unit[3] := -q_unit[3];
        q_unit[4] := -q_unit[4];
      end if;
      v_norm := sqrt(q_unit[2] * q_unit[2] + q_unit[3] * q_unit[3] + q_unit[4] * q_unit[4]);
      angle := 2.0 * atan2(v_norm, q_unit[1]);
      scale := if v_norm > 1.0e-8 then angle / v_norm else 2.0;
      phi[1] := scale * q_unit[2];
      phi[2] := scale * q_unit[3];
      phi[3] := scale * q_unit[4];
    end quaternionLog;

    function quaternionError
      input Real reference[4] "Reference quaternion w,x,y,z";
      input Real measured[4] "Measured quaternion w,x,y,z";
      output Real phi[3] "Small-angle error from reference to measured";

    algorithm
      phi := quaternionLog(quaternionProduct(quaternionConjugate(reference), measured));
    end quaternionError;

    function quaternionNormError
      input Real q[4] "Quaternion w,x,y,z";
      output Real err "Quaternion norm error";

    algorithm
      err := q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4] - 1;
    end quaternionNormError;

    function quaternionDerivative
      input Real q[4] "Quaternion w,x,y,z";
      input Real omega[3] "Body angular velocity [rad/s]";
      input Real qnorm_gain = 1.0 "Quaternion renormalization gain";
      output Real q_dot[4] "Quaternion derivative";
    protected
      Real err "Quaternion norm error";

    algorithm
      err := q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4] - 1;
      q_dot[1] := 0.5 * (-q[2] * omega[1] - q[3] * omega[2] - q[4] * omega[3]) - qnorm_gain * err * q[1];
      q_dot[2] := 0.5 * (q[1] * omega[1] - q[4] * omega[2] + q[3] * omega[3]) - qnorm_gain * err * q[2];
      q_dot[3] := 0.5 * (q[4] * omega[1] + q[1] * omega[2] - q[2] * omega[3]) - qnorm_gain * err * q[3];
      q_dot[4] := 0.5 * (-q[3] * omega[1] + q[2] * omega[2] + q[1] * omega[3]) - qnorm_gain * err * q[4];
    end quaternionDerivative;

    function rotationMatrix
      input Real q[4] "Quaternion w,x,y,z";
      output Real R[3, 3] "Direction cosine matrix, body to world";

    algorithm
      R[1, 1] := 1 - 2 * (q[3] * q[3] + q[4] * q[4]);
      R[1, 2] := 2 * (q[2] * q[3] - q[1] * q[4]);
      R[1, 3] := 2 * (q[2] * q[4] + q[1] * q[3]);
      R[2, 1] := 2 * (q[2] * q[3] + q[1] * q[4]);
      R[2, 2] := 1 - 2 * (q[2] * q[2] + q[4] * q[4]);
      R[2, 3] := 2 * (q[3] * q[4] - q[1] * q[2]);
      R[3, 1] := 2 * (q[2] * q[4] - q[1] * q[3]);
      R[3, 2] := 2 * (q[3] * q[4] + q[1] * q[2]);
      R[3, 3] := 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    end rotationMatrix;

    model Quaternion
      parameter Real q_start[4] = {1, 0, 0, 0} "Initial quaternion w,x,y,z";
      parameter Real qnorm_gain = 1.0 "Quaternion renormalization gain";

      input Real omega[3] "Body angular velocity [rad/s]";
      output Real q[4](start = q_start, each fixed = true) "Quaternion w,x,y,z";
      output Real R[3, 3](start = [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
      ]) "Direction cosine matrix, body to world";
      output Real q_norm_err(start = 0) "Quaternion norm error";

    equation
      q_norm_err = quaternionNormError(q);
      der(q) = quaternionDerivative(q, omega, qnorm_gain);
      R = rotationMatrix(q);
    end Quaternion;
  end SO3;
end LieGroup;
