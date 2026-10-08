within SLAM.Inertial;
// One complete ES15 pose-observation correction, matching ErrorStateFilter.correct_pose.
// World-additive p/v, right-local theta, body accel/gyro biases; Hamilton wxyz output.
// Compile this source together with SPD6Solve.mo for ES15CorrectionSolve below.
// Same-frame Modelica composition (no host numerical solve):
// 1. ES15CorrectionSolve(A=observation_covariance,B=zeros) supplies its valid flag.
// 2. Evaluate this component's innovation_covariance/solve_rhs from the prior state.
// 3. ES15CorrectionSolve(A=innovation_covariance,B=solve_rhs) supplies solved/solve_valid.
// 4. Reevaluate this component with those results; commit only its final next outputs.
// The preview uses solve_valid=0; never feed its diagnostics back as another observation.
// The caller supplies a finite valid nominal state and symmetric positive-definite P.
// Covariance inputs have no replacement/default covariance. Feed every entry per frame.
// WASM's finite-value input boundary rejects NaN/Infinity before this numerical component.
// Outputs are a proposed next frame: rejection preserves every nominal/P entry.
model ES15PoseCorrection
  input Real rotation[3,3] = identity(3);
  input Real position[3] = {0.0,0.0,0.0};
  input Real velocity[3] = {0.0,0.0,0.0};
  input Real accel_bias[3] = {0.0,0.0,0.0};
  input Real gyro_bias[3] = {0.0,0.0,0.0};
  input Real covariance[15,15];
  input Real observed_rotation[3,3] = identity(3);
  input Real observed_position[3] = {0.0,0.0,0.0};
  input Real observation_covariance[6,6];
  input Real solved[6,16];
  input Real solve_valid = 0.0;
  input Real observation_covariance_valid = 0.0;
  input Real accepted_count = 0.0;
  input Real rejected_count = 0.0;
  input Real last_nis = 0.0;
  output Real accepted;
  output Real next_accepted_count; output Real next_rejected_count; output Real next_last_nis;
  output Real next_position[3]; output Real next_velocity[3];
  output Real next_accel_bias[3]; output Real next_gyro_bias[3];
  output Real next_rotation[3,3]; output Real next_quaternion[4];
  output Real next_covariance[15,15];
  output Real H[6,15]; output Real cross_covariance[15,6];
  output Real innovation_covariance[6,6]; output Real solve_rhs[6,16];
  output Real innovation[6]; output Real gain[15,6]; output Real correction[15];
  output Real joseph_covariance[15,15];
protected
  Real gram[3,3]; Real rotation_checks[3,3]; Real determinant;
  Real admissible; Real raw_nis; Real correction_checks[15];
  Real relative[3,3]; Real angle; Real antisymmetric[3]; Real log_scale;
  Real relative_raw_q[4]; Real relative_q[4]; Real relative_q_norm;
  Real relative_q_sine; Real relative_q_scale;
  Real qr; Real qx; Real qy; Real qz;
  Real delta_angle; Real a; Real b; Real W[3,3]; Real increment[3,3];
  Real proposed_rotation[3,3]; Real proposed_accel_bias[3]; Real proposed_gyro_bias[3];
  Real residual_map[15,15]; Real left[15,15]; Real noise_left[15,6];
  Real reset_a; Real reset_b; Real reset_jacobian[3,3];
  Real reset_left[15,15]; Real reset_proposed[15,15];
  Real output_raw_q[4]; Real output_q_norm; Real ow; Real ox; Real oy; Real oz;
equation
  gram = transpose(observed_rotation)*observed_rotation;
  determinant = observed_rotation[1,1]*(observed_rotation[2,2]*observed_rotation[3,3]-observed_rotation[2,3]*observed_rotation[3,2])
              - observed_rotation[1,2]*(observed_rotation[2,1]*observed_rotation[3,3]-observed_rotation[2,3]*observed_rotation[3,1])
              + observed_rotation[1,3]*(observed_rotation[2,1]*observed_rotation[3,2]-observed_rotation[2,2]*observed_rotation[3,1]);
  for i in 1:3 loop
    for j in 1:3 loop
      rotation_checks[i,j] = if noEvent(abs(gram[i,j]-(if i == j then 1.0 else 0.0)) <= 1e-6) then 0.0 else 1.0;
    end for;
  end for;
  admissible = if noEvent(sum(rotation_checks[i,j] for i in 1:3, j in 1:3) < 0.5 and abs(determinant-1.0) <= 1e-6 and observation_covariance_valid > 0.5) then 1.0 else 0.0;
  relative = transpose(rotation)*observed_rotation;
  angle = acos(min(1.0,max(-1.0,(relative[1,1]+relative[2,2]+relative[3,3]-1.0)/2.0)));
  antisymmetric = {relative[3,2]-relative[2,3],relative[1,3]-relative[3,1],relative[2,1]-relative[1,2]};
  // Stable dominant-component Hamilton quaternion, including the pi branch.
  qr = 2.0*sqrt(max(0.0,1.0+relative[1,1]+relative[2,2]+relative[3,3]));
  qx = 2.0*sqrt(max(0.0,1.0+relative[1,1]-relative[2,2]-relative[3,3]));
  qy = 2.0*sqrt(max(0.0,1.0-relative[1,1]+relative[2,2]-relative[3,3]));
  qz = 2.0*sqrt(max(0.0,1.0-relative[1,1]-relative[2,2]+relative[3,3]));
  relative_raw_q = if noEvent(relative[1,1]+relative[2,2]+relative[3,3] > 0.0) then
      {qr/4.0,antisymmetric[1]/max(qr,1e-12),antisymmetric[2]/max(qr,1e-12),antisymmetric[3]/max(qr,1e-12)}
    elseif noEvent(relative[1,1] > relative[2,2] and relative[1,1] > relative[3,3]) then
      {antisymmetric[1]/max(qx,1e-12),qx/4.0,(relative[1,2]+relative[2,1])/max(qx,1e-12),(relative[1,3]+relative[3,1])/max(qx,1e-12)}
    elseif noEvent(relative[2,2] > relative[3,3]) then
      {antisymmetric[2]/max(qy,1e-12),(relative[1,2]+relative[2,1])/max(qy,1e-12),qy/4.0,(relative[2,3]+relative[3,2])/max(qy,1e-12)}
    else {antisymmetric[3]/max(qz,1e-12),(relative[1,3]+relative[3,1])/max(qz,1e-12),(relative[2,3]+relative[3,2])/max(qz,1e-12),qz/4.0};
  relative_q_norm = sqrt(sum(relative_raw_q[k]^2 for k in 1:4));
  relative_q = (if noEvent(relative_raw_q[1] < 0.0) then -1.0 else 1.0)*relative_raw_q/max(relative_q_norm,1e-12);
  relative_q_sine = sqrt(relative_q[2]^2+relative_q[3]^2+relative_q[4]^2);
  relative_q_scale = 2.0*atan2(relative_q_sine,relative_q[1])/max(relative_q_sine,1e-12);
  log_scale = if noEvent(angle < 1e-7) then 0.5 else angle/(2.0*max(sin(angle),1e-12));
  for i in 1:3 loop
    innovation[i] = observed_position[i]-position[i];
    innovation[i+3] = if noEvent(angle < 3.141592653589793-1e-5) then log_scale*antisymmetric[i] else relative_q_scale*relative_q[i+1];
  end for;
  for i in 1:6 loop
    for j in 1:15 loop
      H[i,j] = if i <= 3 and j == i or i > 3 and j == i+3 then 1.0 else 0.0;
      solve_rhs[i,j] = cross_covariance[j,i];
    end for;
    solve_rhs[i,16] = innovation[i];
  end for;
  cross_covariance = covariance*transpose(H);
  innovation_covariance = H*cross_covariance+observation_covariance;
  for i in 1:15 loop
    for j in 1:6 loop
      gain[i,j] = solved[j,i];
    end for;
    correction[i] = sum(gain[i,j]*innovation[j] for j in 1:6);
    correction_checks[i] = if noEvent(abs(correction[i]) <= 1.7976931348623157e308) then 0.0 else 1.0;
  end for;
  raw_nis = sum(innovation[i]*solved[i,16] for i in 1:6);
  next_last_nis = if noEvent(admissible > 0.5) then (if noEvent(abs(raw_nis) <= 1.7976931348623157e308) then raw_nis else 0.0) else last_nis;
  proposed_accel_bias = accel_bias+{correction[10],correction[11],correction[12]};
  proposed_gyro_bias = gyro_bias+{correction[13],correction[14],correction[15]};
  accepted = if noEvent(admissible > 0.5 and solve_valid > 0.5 and raw_nis >= 0.0 and raw_nis <= 22.46 and
    innovation[4]^2+innovation[5]^2+innovation[6]^2 <= 0.35^2 and sum(correction_checks[i] for i in 1:15) < 0.5 and
    sum(proposed_accel_bias[i]^2 for i in 1:3) <= 2.0^2 and sum(proposed_gyro_bias[i]^2 for i in 1:3) <= 0.3^2) then 1.0 else 0.0;
  next_accepted_count = accepted_count+accepted;
  next_rejected_count = rejected_count+1.0-accepted;
  residual_map = identity(15)-gain*H;
  left = residual_map*covariance;
  noise_left = gain*observation_covariance;
  joseph_covariance = left*transpose(residual_map)+noise_left*transpose(gain);
  delta_angle = sqrt(correction[7]^2+correction[8]^2+correction[9]^2);
  a = if noEvent(delta_angle < 1e-7) then 1.0 else sin(delta_angle)/max(delta_angle,1e-7);
  b = if noEvent(delta_angle < 1e-7) then 0.5 else (1.0-cos(delta_angle))/max(delta_angle^2,1e-14);
  W = [0.0,-correction[9],correction[8];correction[9],0.0,-correction[7];-correction[8],correction[7],0.0];
  increment = identity(3)+a*W+b*(W*W);
  proposed_rotation = rotation*increment;
  // Exact right-local reset, including every p/v/bias-attitude cross term.
  reset_a = if noEvent(delta_angle < 1e-6) then 0.5 else (1.0-cos(delta_angle))/max(delta_angle^2,1e-12);
  reset_b = if noEvent(delta_angle < 1e-6) then 1.0/6.0 else (delta_angle-sin(delta_angle))/max(delta_angle^3,1e-18);
  reset_jacobian = identity(3)-reset_a*W+reset_b*(W*W);
  for j in 1:15 loop
    for i in 1:6 loop
      reset_left[i,j] = joseph_covariance[i,j];
    end for;
    for i in 1:3 loop
      reset_left[i+6,j] = sum(reset_jacobian[i,k]*joseph_covariance[k+6,j] for k in 1:3);
    end for;
    for i in 10:15 loop
      reset_left[i,j] = joseph_covariance[i,j];
    end for;
  end for;
  for i in 1:15 loop
    for j in 1:6 loop
      reset_proposed[i,j] = reset_left[i,j];
    end for;
    for j in 1:3 loop
      reset_proposed[i,j+6] = sum(reset_left[i,k+6]*reset_jacobian[j,k] for k in 1:3);
    end for;
    for j in 10:15 loop
      reset_proposed[i,j] = reset_left[i,j];
    end for;
  end for;
  for i in 1:3 loop
    next_position[i] = if noEvent(accepted > 0.5) then position[i]+correction[i] else position[i];
    next_velocity[i] = if noEvent(accepted > 0.5) then velocity[i]+correction[i+3] else velocity[i];
    next_accel_bias[i] = if noEvent(accepted > 0.5) then proposed_accel_bias[i] else accel_bias[i];
    next_gyro_bias[i] = if noEvent(accepted > 0.5) then proposed_gyro_bias[i] else gyro_bias[i];
    for j in 1:3 loop
      next_rotation[i,j] = if noEvent(accepted > 0.5) then proposed_rotation[i,j] else rotation[i,j];
    end for;
  end for;
  for i in 1:15 loop
    for j in 1:15 loop
      next_covariance[i,j] = if noEvent(accepted > 0.5) then 0.5*(reset_proposed[i,j]+reset_proposed[j,i]) else covariance[i,j];
    end for;
  end for;
  ow = 2.0*sqrt(max(0.0,1.0+next_rotation[1,1]+next_rotation[2,2]+next_rotation[3,3]));
  ox = 2.0*sqrt(max(0.0,1.0+next_rotation[1,1]-next_rotation[2,2]-next_rotation[3,3]));
  oy = 2.0*sqrt(max(0.0,1.0-next_rotation[1,1]+next_rotation[2,2]-next_rotation[3,3]));
  oz = 2.0*sqrt(max(0.0,1.0-next_rotation[1,1]-next_rotation[2,2]+next_rotation[3,3]));
  output_raw_q = if noEvent(next_rotation[1,1]+next_rotation[2,2]+next_rotation[3,3] > 0.0) then
      {ow/4.0,(next_rotation[3,2]-next_rotation[2,3])/max(ow,1e-12),(next_rotation[1,3]-next_rotation[3,1])/max(ow,1e-12),(next_rotation[2,1]-next_rotation[1,2])/max(ow,1e-12)}
    elseif noEvent(next_rotation[1,1] > next_rotation[2,2] and next_rotation[1,1] > next_rotation[3,3]) then
      {(next_rotation[3,2]-next_rotation[2,3])/max(ox,1e-12),ox/4.0,(next_rotation[1,2]+next_rotation[2,1])/max(ox,1e-12),(next_rotation[1,3]+next_rotation[3,1])/max(ox,1e-12)}
    elseif noEvent(next_rotation[2,2] > next_rotation[3,3]) then
      {(next_rotation[1,3]-next_rotation[3,1])/max(oy,1e-12),(next_rotation[1,2]+next_rotation[2,1])/max(oy,1e-12),oy/4.0,(next_rotation[2,3]+next_rotation[3,2])/max(oy,1e-12)}
    else {(next_rotation[2,1]-next_rotation[1,2])/max(oz,1e-12),(next_rotation[1,3]+next_rotation[3,1])/max(oz,1e-12),(next_rotation[2,3]+next_rotation[3,2])/max(oz,1e-12),oz/4.0};
  output_q_norm = sqrt(sum(output_raw_q[k]^2 for k in 1:4));
  next_quaternion = (if noEvent(output_raw_q[1] < 0.0) then -1.0 else 1.0)*output_raw_q/max(output_q_norm,1e-12);
end ES15PoseCorrection;
