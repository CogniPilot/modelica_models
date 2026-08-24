within;
package EskfJacobianProof
  "Proof that the rumoca jacobian construct synthesizes ESKF measurement
   Jacobians that match the hand-written filter H on flight math.

   The differentiated retraction (retract_* below) reproduces
   Estimation.StrapdownINS.ESKF.inject: right-perturbation on SE_2(3) via the
   SE23 exponential and group product. Its callees are unqualified siblings
   because the jacobian engine descends only through unqualified in-scope
   functions; each body is copied verbatim from LieGroups, the single deviation
   being that SO(3) left_jacobian's max(theta_sq, eps) NaN guard is written
   theta_sq (a no-op on the branch that reads it, unreached at delta = 0).

   The hand-written H (Hhand) is computed from the SHIPPED LieGroups library by
   qualified calls in the equation section, so the comparison is a parity check
   against the real flight code, not against the local copy."

  // ---- Self-contained SO(3)/SE_2(3), unqualified, faithful to LieGroups ----

  function wedge
    input Real v[3];
    output Real S[3, 3];
  algorithm
    S[1,1] :=  0;     S[1,2] := -v[3];  S[1,3] :=  v[2];
    S[2,1] :=  v[3];  S[2,2] :=  0;     S[2,3] := -v[1];
    S[3,1] := -v[2];  S[3,2] :=  v[1];  S[3,3] :=  0;
  end wedge;

  function to_dcm
    input Real q[4];
    output Real R[3, 3];
  protected
    Real a, b, c, d;
  algorithm
    a := q[1]; b := q[2]; c := q[3]; d := q[4];
    R[1,1] := a*a + b*b - c*c - d*d;
    R[1,2] := 2*(b*c - a*d);
    R[1,3] := 2*(b*d + a*c);
    R[2,1] := 2*(b*c + a*d);
    R[2,2] := a*a - b*b + c*c - d*d;
    R[2,3] := 2*(c*d - a*b);
    R[3,1] := 2*(b*d - a*c);
    R[3,2] := 2*(c*d + a*b);
    R[3,3] := a*a - b*b - c*c + d*d;
  end to_dcm;

  function so3_exp
    input Real v[3];
    output Real q[4];
  protected
    Real theta_sq;
    Real half_theta;
    Real A;
    Real B;
    constant Real eps = 1e-8;
  algorithm
    theta_sq := v[1]^2 + v[2]^2 + v[3]^2;
    if theta_sq < eps then
      B := 1.0 - theta_sq / 8.0;
      A := 0.5 - theta_sq / 48.0;
    else
      half_theta := sqrt(theta_sq) / 2.0;
      B := cos(half_theta);
      A := sin(half_theta) / sqrt(theta_sq);
    end if;
    q[1] := B;
    q[2] := A * v[1];
    q[3] := A * v[2];
    q[4] := A * v[3];
  end so3_exp;

  function so3_left_jacobian
    input Real v[3];
    output Real J[3, 3];
  protected
    Real theta_sq;
    Real A;
    Real B;
    Real theta;
    Real S[3, 3];
    Real S2[3, 3];
    constant Real eps = 1e-2;
  algorithm
    theta_sq := v[1]^2 + v[2]^2 + v[3]^2;
    S := wedge(v);
    S2 := S * S;
    if theta_sq < eps then
      A := 0.5 - theta_sq / 24.0;
      B := 1.0/6.0 - theta_sq / 120.0;
    else
      theta := sqrt(theta_sq);
      A := (1.0 - cos(theta)) / (theta * theta);
      B := (theta - sin(theta)) / (theta * theta * theta);
    end if;
    J := identity(3) + A * S + B * S2;
  end so3_left_jacobian;

  function so3_rotate
    input Real q[4];
    input Real v[3];
    output Real v_out[3];
  protected
    Real R[3, 3];
  algorithm
    R := to_dcm(q);
    v_out := R * v;
  end so3_rotate;

  function so3_quat_product
    input Real q[4];
    input Real p[4];
    output Real r[4];
  algorithm
    r[1] := q[1]*p[1] - q[2]*p[2] - q[3]*p[3] - q[4]*p[4];
    r[2] := q[2]*p[1] + q[1]*p[2] - q[4]*p[3] + q[3]*p[4];
    r[3] := q[3]*p[1] + q[4]*p[2] + q[1]*p[3] - q[2]*p[4];
    r[4] := q[4]*p[1] - q[3]*p[2] + q[2]*p[3] + q[1]*p[4];
  end so3_quat_product;

  function se23_exp
    input Real xi[9];
    output Real X[10];
  protected
    Real J[3, 3];
    Real q[4];
  algorithm
    J := so3_left_jacobian(xi[7:9]);
    q := so3_exp(xi[7:9]);
    X[1:3] := J * xi[1:3];
    X[4:6] := J * xi[4:6];
    X[7:10] := q;
  end se23_exp;

  function se23_product
    input Real X1[10];
    input Real X2[10];
    output Real X[10];
  protected
    Real p_rot[3], v_rot[3];
    Real q_prod[4];
  algorithm
    p_rot := so3_rotate(X1[7:10], X2[1:3]);
    X[1] := p_rot[1] + X1[1];
    X[2] := p_rot[2] + X1[2];
    X[3] := p_rot[3] + X1[3];
    v_rot := so3_rotate(X1[7:10], X2[4:6]);
    X[4] := v_rot[1] + X1[4];
    X[5] := v_rot[2] + X1[5];
    X[6] := v_rot[3] + X1[6];
    q_prod := so3_quat_product(X1[7:10], X2[7:10]);
    X[7] := q_prod[1];
    X[8] := q_prod[2];
    X[9] := q_prod[3];
    X[10] := q_prod[4];
  end se23_product;

  function yaw_from_quat "3-2-1 yaw of a unit quaternion (atan2 form, no clamp)"
    input Real q[4];
    output Real psi;
  algorithm
    psi := atan2(2.0*(q[1]*q[4] + q[2]*q[3]),
                 1.0 - 2.0*(q[3]*q[3] + q[4]*q[4]));
  end yaw_from_quat;

  function ej "j-th standard basis vector of R^15"
    input Integer j;
    output Real e[15];
  algorithm
    e := zeros(15);
    e[j] := 1.0;
  end ej;

  // ---- Measurement functions h(retract(stateBar, delta)) ----

  function retract_pos_z "correctBarometer: world-up (ENU z) position"
    input Real pBar[3];
    input Real vBar[3];
    input Real qBar[4];
    input Real delta[15];
    output Real y;
  protected
    Real ext[10];
    Real corr[10];
    Real out[10];
  algorithm
    ext := cat(1, pBar, vBar, qBar);
    corr := se23_exp(delta[1:9]);
    out := se23_product(ext, corr);
    y := out[3];
  end retract_pos_z;

  function retract_gps_body
    "correctGpsPosition: body-frame predicted position R_bar' p(retract).
     R_bar' is the constant linearization-point rotation, passed in as Rt so
     the differentiated body carries no transpose of a user-call result."
    input Real pBar[3];
    input Real vBar[3];
    input Real qBar[4];
    input Real Rt[3, 3];
    input Real delta[15];
    output Real y[3];
  protected
    Real ext[10];
    Real corr[10];
    Real out[10];
  algorithm
    ext := cat(1, pBar, vBar, qBar);
    corr := se23_exp(delta[1:9]);
    out := se23_product(ext, corr);
    y := Rt * out[1:3];
  end retract_gps_body;

  function retract_yaw "correctMagnetometer: 3-2-1 yaw of retracted attitude"
    input Real pBar[3];
    input Real vBar[3];
    input Real qBar[4];
    input Real delta[15];
    output Real y;
  protected
    Real ext[10];
    Real corr[10];
    Real out[10];
  algorithm
    ext := cat(1, pBar, vBar, qBar);
    corr := se23_exp(delta[1:9]);
    out := se23_product(ext, corr);
    y := yaw_from_quat(out[7:10]);
  end retract_yaw;

  // ---- Proof models: synthesized H vs shipped-library hand H vs FD ----

  model Barometer "correctBarometer measurement-Jacobian parity"
    parameter Real pBar[3] = {12.0, -5.0, 30.0};
    parameter Real qRaw[4] = {0.9238795325, 0.15, -0.22, 0.28};
    parameter Real qBar[4] = qRaw / sqrt(qRaw * qRaw);
    parameter Real vBar[3] = {1.0, 2.0, -0.5};
    parameter Real delta[15] = zeros(15);
    parameter Real h = 1e-6;
    Real Hsynth[1, 15] = jacobian(retract_pos_z(pBar, vBar, qBar, delta), delta);
    Real Rlib[3, 3] = LieGroups.SO3.Quat.to_DCM(qBar);
    Real Hhand[1, 15] = cat(2, {Rlib[3, :]}, zeros(1, 12));
    Real Jfd[1, 15];
    Real maxDiffHand;
    Real maxDiffFd;
    Real sizeH;
    Real clock(start = 0, fixed = true);
  equation
    der(clock) = 0;
    for j in 1:15 loop
      Jfd[1, j] = (retract_pos_z(pBar, vBar, qBar, delta + h*ej(j))
        - retract_pos_z(pBar, vBar, qBar, delta - h*ej(j))) / (2*h);
    end for;
    maxDiffHand = max(abs(Hsynth - Hhand));
    maxDiffFd = max(abs(Hsynth - Jfd));
    sizeH = max(abs(Hsynth));
  end Barometer;

  model GpsPosition "correctGpsPosition measurement-Jacobian parity"
    parameter Real pBar[3] = {12.0, -5.0, 30.0};
    parameter Real qRaw[4] = {0.9238795325, 0.15, -0.22, 0.28};
    parameter Real qBar[4] = qRaw / sqrt(qRaw * qRaw);
    parameter Real vBar[3] = {1.0, 2.0, -0.5};
    parameter Real Rt[3, 3] = transpose(LieGroups.SO3.Quat.to_DCM(qBar));
    parameter Real delta[15] = zeros(15);
    parameter Real h = 1e-6;
    Real Hsynth[3, 15] = jacobian(retract_gps_body(pBar, vBar, qBar, Rt, delta), delta);
    Real Hhand[3, 15] = cat(2, identity(3), zeros(3, 12));
    Real Jfd[3, 15];
    Real maxDiffHand;
    Real maxDiffFd;
    Real sizeH;
    Real clock(start = 0, fixed = true);
  equation
    der(clock) = 0;
    for j in 1:15 loop
      Jfd[:, j] = (retract_gps_body(pBar, vBar, qBar, Rt, delta + h*ej(j))
        - retract_gps_body(pBar, vBar, qBar, Rt, delta - h*ej(j))) / (2*h);
    end for;
    maxDiffHand = max(abs(Hsynth - Hhand));
    maxDiffFd = max(abs(Hsynth - Jfd));
    sizeH = max(abs(Hsynth));
  end GpsPosition;

  model MagnetometerYaw "correctMagnetometer yaw measurement-Jacobian parity"
    parameter Real pBar[3] = {12.0, -5.0, 30.0};
    parameter Real qRaw[4] = {0.9238795325, 0.15, -0.22, 0.28};
    parameter Real qBar[4] = qRaw / sqrt(qRaw * qRaw);
    parameter Real vBar[3] = {1.0, 2.0, -0.5};
    parameter Real delta[15] = zeros(15);
    parameter Real h = 1e-6;
    Real Hsynth[1, 15] = jacobian(retract_yaw(pBar, vBar, qBar, delta), delta);
    Real euler[3] = LieGroups.SO3.EulerB321.from_Quat(qBar);
    Real cosPitch = cos(euler[2]);
    Real Hhand[1, 15] = cat(2, zeros(1, 6),
      {{0.0, sin(euler[3])/cosPitch, cos(euler[3])/cosPitch}}, zeros(1, 6));
    Real Jfd[1, 15];
    Real maxDiffHand;
    Real maxDiffFd;
    Real sizeH;
    Real clock(start = 0, fixed = true);
  equation
    der(clock) = 0;
    for j in 1:15 loop
      Jfd[1, j] = (retract_yaw(pBar, vBar, qBar, delta + h*ej(j))
        - retract_yaw(pBar, vBar, qBar, delta - h*ej(j))) / (2*h);
    end for;
    maxDiffHand = max(abs(Hsynth - Hhand));
    maxDiffFd = max(abs(Hsynth - Jfd));
    sizeH = max(abs(Hsynth));
  end MagnetometerYaw;
end EskfJacobianProof;
