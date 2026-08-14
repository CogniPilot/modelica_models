within Estimation;
block ComplementaryAttitude "Quaternion attitude estimate corrected by gravity"
  parameter Real samplePeriod(unit="s") = 0.005;
  parameter Real gravity(unit="m/s2") = 9.81;
  parameter Real correctionKp = 0.35;
  parameter Real correctionKi = 0.02;
  parameter Real accelMinG = 0.6;
  parameter Real accelMaxG = 1.4;

  input Real gyro_rad_s[3](start={0.0, 0.0, 0.0});
  input Real accel_m_s2[3](start={0.0, 0.0, gravity});
  input Boolean reset(start=true);

  discrete output Real quaternion[4](start={1.0, 0.0, 0.0, 0.0});
  discrete output Real euler_rad[3](start={0.0, 0.0, 0.0});

protected
  discrete Real gyroBias_rad_s[3](start={0.0, 0.0, 0.0});
  discrete Real accelNorm(start=gravity);
  discrete Boolean accelValid(start=true);
  discrete Real normalizedAccel[3](start={0.0, 0.0, 1.0});
  discrete Real estimatedGravity[3](start={0.0, 0.0, 1.0});
  discrete Real correctionError[3](start={0.0, 0.0, 0.0});
  discrete Real correctedGyro[3](start={0.0, 0.0, 0.0});
  discrete Real unnormalizedQuaternion[4](start={1.0, 0.0, 0.0, 0.0});
  discrete Real quaternionNorm(start=1.0);
  discrete Real resetRoll(start=0.0);
  discrete Real resetPitch(start=0.0);
  discrete Real halfRoll(start=0.0);
  discrete Real halfPitch(start=0.0);

algorithm
  when sample(0.0, samplePeriod) then
    accelNorm := MathUtilities.norm3(accel_m_s2);
    accelValid := accelNorm >= accelMinG * gravity and
      accelNorm <= accelMaxG * gravity;

    if reset then
      resetRoll := atan2(accel_m_s2[2], accel_m_s2[3]);
      resetPitch := atan2(
        -accel_m_s2[1],
        sqrt(accel_m_s2[2] * accel_m_s2[2] +
          accel_m_s2[3] * accel_m_s2[3]));
      halfRoll := 0.5 * resetRoll;
      halfPitch := 0.5 * resetPitch;
      quaternion[1] := cos(halfPitch) * cos(halfRoll);
      quaternion[2] := cos(halfPitch) * sin(halfRoll);
      quaternion[3] := sin(halfPitch) * cos(halfRoll);
      quaternion[4] := -sin(halfPitch) * sin(halfRoll);
      gyroBias_rad_s := {0.0, 0.0, 0.0};
      normalizedAccel := {0.0, 0.0, 1.0};
      estimatedGravity := {0.0, 0.0, 1.0};
      correctionError := {0.0, 0.0, 0.0};
      correctedGyro := gyro_rad_s;
      unnormalizedQuaternion := quaternion;
      quaternionNorm := 1.0;
    else
      if accelValid then
        normalizedAccel := accel_m_s2 / accelNorm;
        estimatedGravity[1] := 2.0 *
          (pre(quaternion[2]) * pre(quaternion[4]) -
            pre(quaternion[1]) * pre(quaternion[3]));
        estimatedGravity[2] := 2.0 *
          (pre(quaternion[1]) * pre(quaternion[2]) +
            pre(quaternion[3]) * pre(quaternion[4]));
        estimatedGravity[3] :=
          pre(quaternion[1]) * pre(quaternion[1]) -
          pre(quaternion[2]) * pre(quaternion[2]) -
          pre(quaternion[3]) * pre(quaternion[3]) +
          pre(quaternion[4]) * pre(quaternion[4]);
        correctionError[1] := normalizedAccel[2] * estimatedGravity[3] -
          normalizedAccel[3] * estimatedGravity[2];
        correctionError[2] := normalizedAccel[3] * estimatedGravity[1] -
          normalizedAccel[1] * estimatedGravity[3];
        correctionError[3] := normalizedAccel[1] * estimatedGravity[2] -
          normalizedAccel[2] * estimatedGravity[1];
        gyroBias_rad_s := pre(gyroBias_rad_s) +
          correctionKi * correctionError * samplePeriod;
      else
        normalizedAccel := pre(normalizedAccel);
        estimatedGravity := pre(estimatedGravity);
        correctionError := {0.0, 0.0, 0.0};
        gyroBias_rad_s := pre(gyroBias_rad_s);
      end if;

      correctedGyro := gyro_rad_s + gyroBias_rad_s +
        correctionKp * correctionError;
      unnormalizedQuaternion[1] := pre(quaternion[1]) + 0.5 *
        (-pre(quaternion[2]) * correctedGyro[1] -
          pre(quaternion[3]) * correctedGyro[2] -
          pre(quaternion[4]) * correctedGyro[3]) * samplePeriod;
      unnormalizedQuaternion[2] := pre(quaternion[2]) + 0.5 *
        (pre(quaternion[1]) * correctedGyro[1] +
          pre(quaternion[3]) * correctedGyro[3] -
          pre(quaternion[4]) * correctedGyro[2]) * samplePeriod;
      unnormalizedQuaternion[3] := pre(quaternion[3]) + 0.5 *
        (pre(quaternion[1]) * correctedGyro[2] -
          pre(quaternion[2]) * correctedGyro[3] +
          pre(quaternion[4]) * correctedGyro[1]) * samplePeriod;
      unnormalizedQuaternion[4] := pre(quaternion[4]) + 0.5 *
        (pre(quaternion[1]) * correctedGyro[3] +
          pre(quaternion[2]) * correctedGyro[2] -
          pre(quaternion[3]) * correctedGyro[1]) * samplePeriod;
      quaternionNorm := MathUtilities.norm4(unnormalizedQuaternion);
      if quaternionNorm > 1.0e-3 then
        quaternion := unnormalizedQuaternion / quaternionNorm;
      else
        quaternion := {1.0, 0.0, 0.0, 0.0};
      end if;
    end if;

    euler_rad[1] := atan2(
      2.0 * (quaternion[1] * quaternion[2] +
        quaternion[3] * quaternion[4]),
      1.0 - 2.0 *
        (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]));
    euler_rad[2] := asin(MathUtilities.clip(
      2.0 * (quaternion[1] * quaternion[3] -
        quaternion[4] * quaternion[2]),
      -1.0,
      1.0));
    euler_rad[3] := atan2(
      2.0 * (quaternion[1] * quaternion[4] +
        quaternion[2] * quaternion[3]),
      1.0 - 2.0 *
        (quaternion[3] * quaternion[3] + quaternion[4] * quaternion[4]));
  end when;
end ComplementaryAttitude;
