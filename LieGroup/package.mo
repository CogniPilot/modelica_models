package LieGroup
  package SO3
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
