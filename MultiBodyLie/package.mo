within;

package MultiBodyLie "MSL MultiBody rewritten on articulated-body connectors (layout and names follow Modelica.Mechanics.MultiBody)"
  // Conventions (see Interfaces.Frame). Spatial motion {w; v} and force {t; f} are resolved in the frame and taken about
  // its origin; gravity acts on each Body as a force at its centre of mass, world.gravityAcceleration(r) as in MSL. In each
  // connection set exactly one component owns the frame's kinematics (World, a joint's child-side frame, a rigid
  // translation's far frame, a free-flight root Body) and receives the flows; every other component defines its flow
  // contributions (f, t, pA, IA) and dv = 0.

  model World "Inertial root and gravity field (no animation)"
    parameter Boolean enableAnimation = true "Ignored: no animation";
    parameter Types.GravityTypes gravityType = Types.GravityTypes.UniformGravity "Type of gravity field";
    parameter Real g = 9.80665 "Gravity constant [m/s2] (MSL default Modelica.Constants.g_n)";
    parameter Real n[3] = {0, -1, 0} "Direction of gravity (MSL default -y)";
    parameter Real mu = 3.986004418e14 "Gravity field constant [m3/s2] (MSL default: the earth's)";
    parameter Real gravitySphereDiameter = 12742000 "Ignored: no animation";
    parameter Real axisLength = 1 "Ignored: no animation";
    parameter Real nominalLength = 1 "Ignored: no animation";
    parameter Boolean animateWorld = true "Ignored: no animation";
    // MSL World defaults that component animation inputs bind to (names and formulas as MSL; no animation here).
    parameter Real defaultAxisLength = nominalLength / 5 "Ignored: no animation";
    parameter Real defaultJointLength = nominalLength / 10 "Ignored: no animation";
    parameter Real defaultJointWidth = nominalLength / 20 "Ignored: no animation";
    parameter Real defaultForceLength = nominalLength / 10 "Ignored: no animation";
    parameter Real defaultForceWidth = nominalLength / 20 "Ignored: no animation";
    parameter Real defaultBodyDiameter = nominalLength / 9 "Ignored: no animation";
    parameter Real defaultWidthFraction = 20 "Ignored: no animation";
    parameter Real defaultArrowDiameter = nominalLength / 40 "Ignored: no animation";
    parameter Real defaultFrameDiameterFraction = 40 "Ignored: no animation";
    parameter Real defaultSpecularCoefficient(min = 0) = 0.7 "Ignored: no animation";
    parameter Real defaultN_to_m(min = 0) = 1000 "Ignored: no animation";
    parameter Real defaultNm_to_m(min = 0) = 1000 "Ignored: no animation";
    parameter Boolean animateGravity = true "Ignored: no animation";
    // As MSL: every Body takes its gravity from this function at its centre of mass, so a redeclared field applies
    // whatever gravityType says.
    replaceable function gravityAcceleration = Forces.Internal.standardGravityAcceleration(gravityType = gravityType, g = g * n / sqrt(n * n), mu = mu)
      constrainedby Interfaces.partialGravityAcceleration;
    Interfaces.Frame_b frame_b;
  equation
    Connections.root(frame_b.R);
    frame_b.r_0 = zeros(3);
    frame_b.R.T = identity(3);
    frame_b.R.w = zeros(3);
    frame_b.v = zeros(3);
    frame_b.a = zeros(6);
    frame_b.dI = zeros(21);
  end World;

  package Frames
    record Orientation "MSL-compatible orientation: T maps world to frame coordinates; w is the angular velocity in frame coordinates"
      Real T[3, 3];
      Real w[3];
      encapsulated function equalityConstraint "Residue of two orientations that should be equal (overconstrained connector)"
        import MultiBodyLie.Frames.Orientation;
        import MultiBodyLie.Frames.orientationResidue;
        input Orientation R1;
        input Orientation R2;
        output Real residue[3];
      algorithm
        residue := orientationResidue(R1.T, R2.T);
      end equalityConstraint;
    end Orientation;

    function orientationResidue "Zero only when T1 and T2 are the same orientation: the rotation vector of the relative rotation"
      // log of T1 T2' (frame_2 -> frame_1): |residue| is the relative angle, pi for an exact half turn. The earlier
      // half-skew form 0.5 (Tr - Tr') was zero for every 180 degree rotation (INIT-HALF-TURN1).
      input Real T1[3, 3];
      input Real T2[3, 3];
      output Real residue[3];
    algorithm
      residue := LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.from_DCM(T1 * transpose(T2)));
    end orientationResidue;

    function resolve1 "Vector resolved in the frame of R -> world"
      input Orientation R;
      input Real v2[3];
      output Real v1[3];
    algorithm
      v1 := transpose(R.T) * v2;
    end resolve1;

    function resolve2 "Vector in world coordinates -> resolved in the frame of R"
      input Orientation R;
      input Real v1[3];
      output Real v2[3];
    algorithm
      v2 := R.T * v1;
    end resolve2;
  end Frames;

  package Types
    type GravityTypes = enumeration(NoGravity "No gravity field", UniformGravity "Uniform gravity field", PointGravity "Point gravity field");
    type ResolveInFrameB = enumeration(world "Resolve in world frame", frame_b "Resolve in frame_b", frame_resolve "Resolve in frame_resolve (unsupported)");
  end Types;

  package Interfaces
    connector Frame "MSL Frame variables plus the articulated-body quantities, balanced pairwise"
      Real r_0[3] "Origin, world";
      Frames.Orientation R;
      flow Real f[3] "Cut force on the component, frame";
      flow Real t[3] "Cut torque on the component about the origin, frame";
      Real v[3] "Velocity of the origin, frame";
      flow Real dv[3] "Balances v; zero for every non-owner";
      Real a[6] "Spatial acceleration, frame";
      flow Real pA[6] "Bias wrench contribution";
      Real dI[21] "Balances IA; no physics, zero";
      flow Real IA[21] "Articulated inertia contribution, symmetric 6x6 packed (Math.packSym)";
    end Frame;
    connector Frame_a = Frame;
    connector Frame_b = Frame;

    partial function partialGravityAcceleration "Interface of world.gravityAcceleration (MSL name)"
      input Real r[3] "Position from the world frame to the point, world";
      output Real gravity[3] "Gravity acceleration at r, world";
    end partialGravityAcceleration;
  end Interfaces;

  package Math


    function crm "Spatial motion cross product matrix: crm(V) * m = V x m"
      input Real V[6];
      output Real X[6, 6];
    algorithm
      X := zeros(6, 6);
      X[1:3, 1:3] := LieGroups.SO3.Quat.wedge(V[1:3]);
      X[4:6, 1:3] := LieGroups.SO3.Quat.wedge(V[4:6]);
      X[4:6, 4:6] := LieGroups.SO3.Quat.wedge(V[1:3]);
    end crm;

    function motionTransform "Spatial transform parent -> child: child <- parent rotation E, child origin at r (parent coordinates)"
      input Real E[3, 3];
      input Real r[3];
      output Real X[6, 6];
    algorithm
      X := zeros(6, 6);
      X[1:3, 1:3] := E;
      X[4:6, 1:3] := -E * LieGroups.SO3.Quat.wedge(r);
      X[4:6, 4:6] := E;
    end motionTransform;

    function translateInertia "X' S X for the pure translation X = motionTransform(identity(3), r), on packed symmetric 6x6"
      // With S = [A, B; B', C] and X = [I, 0; -[r]x, I]: C' = C, B' = B + [r]x C, A' = A + [r]x B' - B [r]x - [r]x C [r]x.
      // About 110 multiplications instead of two dense 6x6 products.
      input Real p[21];
      input Real r[3];
      output Real q[21];
    protected
      Real A[3, 3] = {{p[1], p[2], p[3]}, {p[2], p[7], p[8]}, {p[3], p[8], p[12]}};
      Real B[3, 3] = {{p[4], p[5], p[6]}, {p[9], p[10], p[11]}, {p[13], p[14], p[15]}};
      Real C[3, 3] = {{p[16], p[17], p[18]}, {p[17], p[19], p[20]}, {p[18], p[20], p[21]}};
      Real rx[3, 3] = LieGroups.SO3.Quat.wedge(r);
      Real rxC[3, 3] = rx * C;
      Real Bn[3, 3] = B + rxC;
      Real An[3, 3] = A + rx * transpose(B) - B * rx - rxC * rx;
    algorithm
      q := {An[1, 1], An[1, 2], An[1, 3], Bn[1, 1], Bn[1, 2], Bn[1, 3], An[2, 2], An[2, 3], Bn[2, 1], Bn[2, 2], Bn[2, 3],
            An[3, 3], Bn[3, 1], Bn[3, 2], Bn[3, 3], p[16], p[17], p[18], p[19], p[20], p[21]};
    end translateInertia;

    function rotateInertia "X' S X for the pure rotation X = motionTransform(R, zeros(3)) = diag(R, R), on packed symmetric 6x6"
      // Blockwise: A' = R' A R, B' = R' B R, C' = R' C R (four 3x3 congruences instead of two dense 6x6 products).
      input Real p[21];
      input Real R[3, 3];
      output Real q[21];
    protected
      Real A[3, 3] = transpose(R) * {{p[1], p[2], p[3]}, {p[2], p[7], p[8]}, {p[3], p[8], p[12]}} * R;
      Real B[3, 3] = transpose(R) * {{p[4], p[5], p[6]}, {p[9], p[10], p[11]}, {p[13], p[14], p[15]}} * R;
      Real C[3, 3] = transpose(R) * {{p[16], p[17], p[18]}, {p[17], p[19], p[20]}, {p[18], p[20], p[21]}} * R;
    algorithm
      q := {A[1, 1], A[1, 2], A[1, 3], B[1, 1], B[1, 2], B[1, 3], A[2, 2], A[2, 3], B[2, 1], B[2, 2], B[2, 3],
            A[3, 3], B[3, 1], B[3, 2], B[3, 3], C[1, 1], C[1, 2], C[1, 3], C[2, 2], C[2, 3], C[3, 3]};
    end rotateInertia;

    function spatialInertia "Spatial inertia about the frame origin of mass m at c with inertia I about the centre of mass"
      input Real m;
      input Real c[3];
      input Real I[3, 3];
      output Real Is[6, 6];
    protected
      Real cx[3, 3] = LieGroups.SO3.Quat.wedge(c);
    algorithm
      Is := zeros(6, 6);
      Is[1:3, 1:3] := I + m * cx * transpose(cx);
      Is[1:3, 4:6] := m * cx;
      Is[4:6, 1:3] := m * transpose(cx);
      Is[4:6, 4:6] := m * identity(3);
    end spatialInertia;


    function packSym "Upper triangle of a symmetric 6x6, row-major (21 entries)"
      input Real M[6, 6];
      output Real p[21];
    algorithm
      p := {M[1, 1], M[1, 2], M[1, 3], M[1, 4], M[1, 5], M[1, 6], M[2, 2], M[2, 3], M[2, 4], M[2, 5], M[2, 6],
            M[3, 3], M[3, 4], M[3, 5], M[3, 6], M[4, 4], M[4, 5], M[4, 6], M[5, 5], M[5, 6], M[6, 6]};
    end packSym;

    function unpackSym "Symmetric 6x6 from its packed upper triangle"
      input Real p[21];
      output Real M[6, 6];
    algorithm
      M := {{p[1], p[2], p[3], p[4], p[5], p[6]},
            {p[2], p[7], p[8], p[9], p[10], p[11]},
            {p[3], p[8], p[12], p[13], p[14], p[15]},
            {p[4], p[9], p[13], p[16], p[17], p[18]},
            {p[5], p[10], p[14], p[17], p[19], p[20]},
            {p[6], p[11], p[15], p[18], p[20], p[21]}};
    end unpackSym;


    function solveSPD "Solve A x = b for a symmetric positive definite A (Cholesky); explicit for the compiler"
      input Real A[:, size(A, 1)];
      input Real b[size(A, 1)];
      output Real x[size(A, 1)];
    protected
      Integer n = size(A, 1);
      Real L[size(A, 1), size(A, 1)];
      Real s;
    algorithm
      L := zeros(n, n);
      for j in 1:n loop
        s := A[j, j];
        for k in 1:j - 1 loop
          s := s - L[j, k] ^ 2;
        end for;
        L[j, j] := sqrt(s);
        for i in j + 1:n loop
          s := A[i, j];
          for k in 1:j - 1 loop
            s := s - L[i, k] * L[j, k];
          end for;
          L[i, j] := s / L[j, j];
        end for;
      end for;
      x := b;
      for i in 1:n loop
        for k in 1:i - 1 loop
          x[i] := x[i] - L[i, k] * x[k];
        end for;
        x[i] := x[i] / L[i, i];
      end for;
      for i in n:-1:1 loop
        for k in i + 1:n loop
          x[i] := x[i] - L[k, i] * x[k];
        end for;
        x[i] := x[i] / L[i, i];
      end for;
    end solveSPD;
  end Math;

  package Parts
    model Body "Rigid body at frame_a; a free-flight root when nothing roots its frame (MSL names)"
      outer MultiBodyLie.World world;
      Interfaces.Frame_a frame_a;
      parameter Boolean animation = true "Ignored: no animation";
      parameter Real r_CM[3] = zeros(3);
      // MSL animation surface (names, variability and default bindings as MSL Parts.Body; values ignored).
      parameter Real sphereDiameter = world.defaultBodyDiameter "Ignored: no animation";
      input Integer sphereColor[3] = {0, 128, 255} "Ignored: no animation (MSL Types.Defaults.BodyColor)";
      parameter Real cylinderDiameter = sphereDiameter / 3 "Ignored: no animation (MSL Types.Defaults.BodyCylinderDiameterFraction = 3)";
      input Integer cylinderColor[3] = sphereColor "Ignored: no animation";
      input Real specularCoefficient = world.defaultSpecularCoefficient "Ignored: no animation";
      parameter Real m = 1;
      parameter Real I_11 = 0.001, I_22 = 0.001, I_33 = 0.001, I_21 = 0, I_31 = 0, I_32 = 0;
      parameter Boolean angles_fixed = false "Root only: fix the start orientation (angles_start = 0 supported)";
      parameter Boolean w_0_fixed = false "Root only: fix the start angular velocity";
      parameter Real w_0_start[3] = zeros(3);
      final parameter Real Is[6, 6] = Math.spatialInertia(m, r_CM, {{I_11, I_21, I_31}, {I_21, I_22, I_32}, {I_31, I_32, I_33}});
      Real r_0[3](start = zeros(3)) "Origin of frame_a, world";
      Real v_0[3](start = zeros(3)) "Velocity of frame_a's origin, world";
      Real a_0[3] "Acceleration of frame_a's origin, world";
      Real w_a[3](start = w_0_start, each fixed = w_0_fixed) "Angular velocity, frame_a";
      Real Q[4](start = {1, 0, 0, 0}) "Root orientation, frame -> world quaternion";
      // Gravity in a binding: the same expression in the equation section read the World class default instead of the
      // inner modifier under rumoca b3afe4cb (reported, unminimised).
      Real g_0[3] = world.gravityAcceleration(frame_a.r_0 + transpose(frame_a.R.T) * r_CM) "Gravity acceleration at the centre of mass, world (MSL name)";
    protected
      Real V[6] = cat(1, frame_a.R.w, frame_a.v);
      Real h[6] = Is * V "Spatial momentum";
      Real gF[3] = m * (frame_a.R.T * g_0) "Gravity force, frame_a";
      Real bias[6] = cat(1, cross(V[1:3], h[1:3]) + cross(V[4:6], h[4:6]) - cross(r_CM, gF), cross(V[1:3], h[4:6]) - gF) "v x* (I v) minus the gravity wrench about the origin";
    initial equation
      // As MSL: angles_fixed fixes frame_a's orientation whichever body the tool picks as the root
      // (angles_start = 0 supported: the start orientation is the world's).
      if angles_fixed then
        if Connections.isRoot(frame_a.R) then
          // The whole quaternion: Q[2:4] = 0 alone left Q[1] at its start value, which is 0 for a half-turn start,
          // so the root started at the zero quaternion (NaN after normalisation).
          Q = {1, 0, 0, 0};
        else
          zeros(3) = Frames.orientationResidue(frame_a.R.T, identity(3)) "unambiguous: a half turn is not a solution";
        end if;
      elseif Connections.isRoot(frame_a.R) then
        // Fix the quaternion's free scale, so that orientation constraints set elsewhere in the tree (a non-root
        // angles_fixed body) determine Q instead of leaving a null direction in the initial solve.
        Q * Q = 1;
      end if;
    equation
      Connections.potentialRoot(frame_a.R);
      if Connections.isRoot(frame_a.R) then
        frame_a.r_0 = r_0;
        frame_a.R.T = transpose(LieGroups.SO3.Quat.to_DCM(LieGroups.SO3.Quat.normalize(Q)));
        frame_a.R.w = w_a;
        frame_a.v = frame_a.R.T * v_0;
        frame_a.dI = zeros(21);
        frame_a.a = -Math.solveSPD(Is - Math.unpackSym(frame_a.IA), bias - frame_a.pA) "explicit: the whole tree's articulated inertia at the root";
        der(r_0) = v_0;
        der(Q) = LieGroups.SO3.Quat.kinematics(Q, w_a);
        der(w_a) = frame_a.a[1:3];
        der(v_0) = a_0;
      else
        frame_a.IA = Math.packSym(Is);
        frame_a.pA = bias;
        frame_a.dv = zeros(3);
        cat(1, frame_a.t, frame_a.f) = Is * frame_a.a + bias;
        r_0 = frame_a.r_0;
        v_0 = transpose(frame_a.R.T) * frame_a.v;
        w_a = frame_a.R.w;
        Q = {1, 0, 0, 0};
      end if;
      a_0 = transpose(frame_a.R.T) * (frame_a.a[4:6] + cross(frame_a.R.w, frame_a.v));
    end Body;

    model FixedTranslation "Rigid offset r from frame_a to frame_b (frame_a coordinates); either direction"
      Interfaces.Frame_a frame_a;
      Interfaces.Frame_b frame_b;
      parameter Boolean animation = true "Ignored: no animation";
      parameter Real r[3] = zeros(3);
    equation
      // Pure translation: X a = {a1, a2 - r x a1}, X' p = {p1 + r x p2, p2}, X' S X by Math.translateInertia.
      Connections.branch(frame_a.R, frame_b.R);
      if Connections.rooted(frame_a.R) then
        frame_b.r_0 = frame_a.r_0 + transpose(frame_a.R.T) * r;
        frame_b.R.T = frame_a.R.T;
        frame_b.R.w = frame_a.R.w;
        frame_b.v = frame_a.v + cross(frame_a.R.w, r);
        frame_b.a = cat(1, frame_a.a[1:3], frame_a.a[4:6] - cross(r, frame_a.a[1:3]));
        frame_b.dI = zeros(21);
        frame_a.IA = -Math.translateInertia(frame_b.IA, r);
        frame_a.pA = -cat(1, frame_b.pA[1:3] + cross(r, frame_b.pA[4:6]), frame_b.pA[4:6]);
        cat(1, frame_a.t, frame_a.f) = Math.unpackSym(frame_a.IA) * frame_a.a + frame_a.pA;
        frame_a.dv = zeros(3);
      else
        frame_a.r_0 = frame_b.r_0 - transpose(frame_b.R.T) * r;
        frame_a.R.T = frame_b.R.T;
        frame_a.R.w = frame_b.R.w;
        frame_a.v = frame_b.v - cross(frame_b.R.w, r);
        frame_a.a = cat(1, frame_b.a[1:3], frame_b.a[4:6] + cross(r, frame_b.a[1:3]));
        frame_a.dI = zeros(21);
        frame_b.IA = -Math.translateInertia(frame_a.IA, -r);
        frame_b.pA = -cat(1, frame_a.pA[1:3] - cross(r, frame_a.pA[4:6]), frame_a.pA[4:6]);
        cat(1, frame_b.t, frame_b.f) = Math.unpackSym(frame_b.IA) * frame_b.a + frame_b.pA;
        frame_b.dv = zeros(3);
      end if;
    end FixedTranslation;

    model BodyShape "Rigid body with frame_a and frame_b (r from a to b) and mass at r_CM (MSL names)"
      Interfaces.Frame_a frame_a;
      Interfaces.Frame_b frame_b;
      parameter Boolean animation = true "Ignored: no animation";
      parameter Real r[3] = zeros(3);
      parameter Real r_CM[3] = zeros(3);
      parameter Real m = 1;
      parameter Real I_11 = 0.001, I_22 = 0.001, I_33 = 0.001, I_21 = 0, I_31 = 0, I_32 = 0;
      Real r_0[3] "Origin of frame_a, world";
      Real v_0[3] "Velocity of frame_a's origin, world";
      Real a_0[3];
      FixedTranslation frameTranslation(r = r);
      Body body(r_CM = r_CM, m = m, I_11 = I_11, I_22 = I_22, I_33 = I_33, I_21 = I_21, I_31 = I_31, I_32 = I_32);
    equation
      r_0 = frame_a.r_0;
      v_0 = body.v_0;
      a_0 = body.a_0;
      connect(frame_a, frameTranslation.frame_a);
      connect(frame_b, frameTranslation.frame_b);
      connect(frame_a, body.frame_a);
    end BodyShape;
  end Parts;

  package Joints
    model Revolute "Revolute joint about n, optionally driven through its axis flange; either direction (MSL names)"
      Modelica.Mechanics.Rotational.Interfaces.Flange_a axis "Always present; unconnected means tau = 0 (useAxisFlange kept for MSL compatibility)";
      Modelica.Mechanics.Rotational.Interfaces.Flange_b support "Joint housing on frame_a: angle 0; its reaction is internal to the joint torque";
      Interfaces.Frame_a frame_a;
      Interfaces.Frame_b frame_b;
      parameter Boolean useAxisFlange = false;
      parameter Boolean animation = true "Ignored: no animation";
      parameter Real n[3] = {0, 0, 1};
      parameter StateSelect stateSelect = StateSelect.prefer;
      final parameter Real e[3] = n / sqrt(n * n);
      Real phi(start = 0, stateSelect = stateSelect) "Rotation of frame_b relative to frame_a about e";
      Real w(start = 0, stateSelect = stateSelect);
      Real a "= der(w)";
      Real tau "Driving torque";
    protected
      // Pure rotation: X = diag(E, E) (frame_b <- frame_a), applied blockwise; X' S X by Math.rotateInertia.
      Real E[3, 3] = transpose(LieGroups.SO3.Dcm.exp_map(e * phi)) "frame_b <- frame_a";
      Real S[6] "Motion subspace in child coordinates";
      Real IAc[6, 6];
      Real pAc[6];
      Real Vc[6];
      Real ap[6];
      Real c[6];
      Real U[6];
      Real D;
      Real u;
      Real Ia[6, 6];
      Real pa[6];
    equation
      Connections.branch(frame_a.R, frame_b.R);
      axis.phi = phi;
      support.phi = 0;
      tau = axis.tau;
      w = der(phi);
      a = der(w);
      if Connections.rooted(frame_a.R) then
        S = cat(1, e, zeros(3));
        frame_b.r_0 = frame_a.r_0;
        frame_b.R.T = E * frame_a.R.T;
        frame_b.R.w = E * frame_a.R.w + e * w;
        frame_b.v = E * frame_a.v;
        Vc = cat(1, frame_b.R.w, frame_b.v);
        IAc = -Math.unpackSym(frame_b.IA);
        pAc = -frame_b.pA;
        ap = cat(1, E * frame_a.a[1:3], E * frame_a.a[4:6]);
        frame_b.a = ap + c + S * a;
        frame_b.dI = zeros(21);
        frame_a.IA = Math.rotateInertia(Math.packSym(Ia), E);
        frame_a.pA = cat(1, transpose(E) * pa[1:3], transpose(E) * pa[4:6]);
        cat(1, frame_a.t, frame_a.f) = Math.unpackSym(frame_a.IA) * frame_a.a + frame_a.pA;
        frame_a.dv = zeros(3);
      else
        S = cat(1, -e, zeros(3));
        frame_a.r_0 = frame_b.r_0;
        frame_a.R.T = transpose(E) * frame_b.R.T;
        frame_a.R.w = transpose(E) * frame_b.R.w - e * w;
        frame_a.v = transpose(E) * frame_b.v;
        Vc = cat(1, frame_a.R.w, frame_a.v);
        IAc = -Math.unpackSym(frame_a.IA);
        pAc = -frame_a.pA;
        ap = cat(1, transpose(E) * frame_b.a[1:3], transpose(E) * frame_b.a[4:6]);
        frame_a.a = ap + c + S * a;
        frame_a.dI = zeros(21);
        frame_b.IA = Math.rotateInertia(Math.packSym(Ia), transpose(E));
        frame_b.pA = cat(1, E * pa[1:3], E * pa[4:6]);
        cat(1, frame_b.t, frame_b.f) = Math.unpackSym(frame_b.IA) * frame_b.a + frame_b.pA;
        frame_b.dv = zeros(3);
      end if;
      c = cat(1, cross(Vc[1:3], S[1:3] * w), cross(Vc[4:6], S[1:3] * w)) "crm(Vc) (S w) with S = {+-e, 0}";
      U = IAc * S;
      D = S * U;
      u = tau - S * pAc;
      Ia = IAc - {{U[i] * U[j] for j in 1:6} for i in 1:6} / D;
      pa = pAc + Ia * c + U * (u / D);
      a = (u - U * (ap + c)) / D;
    end Revolute;

    model Prismatic "Prismatic joint along n, optionally driven through its axis flange; either direction (MSL names)"
      // MSL Joints.Prismatic: frame_b.r_0 = frame_a.r_0 + resolve1(frame_a.R, e s), frame_b.R = frame_a.R, f = -e frame_b.f.
      // Articulated form as Revolute, with the motion subspace S = {0; e} and the pure translation X = motionTransform(I, e s)
      // (X a = {a1, a2 - r x a1}, X' p = {p1 + r x p2, p2}, X' S X by Math.translateInertia, as FixedTranslation).
      outer MultiBodyLie.World world;
      Modelica.Mechanics.Translational.Interfaces.Flange_a axis "Always present; unconnected means f = 0 (useAxisFlange kept for MSL compatibility)";
      Modelica.Mechanics.Translational.Interfaces.Flange_b support "Drive support, assumed fixed in the world frame (NOT in the joint), as MSL: s = 0; its reaction does not enter frame_a or frame_b";
      Interfaces.Frame_a frame_a;
      Interfaces.Frame_b frame_b;
      parameter Boolean useAxisFlange = false;
      parameter Boolean animation = true "Ignored: no animation";
      parameter Real n[3] = {1, 0, 0} "Axis of translation resolved in frame_a (= same as in frame_b)";
      parameter Real boxWidthDirection[3] = {0, 1, 0} "Ignored: no animation";
      parameter Real boxWidth = world.defaultJointWidth "Ignored: no animation";
      parameter Real boxHeight = boxWidth "Ignored: no animation";
      input Integer boxColor[3] = {255, 0, 0} "Ignored: no animation (MSL Types.Defaults.JointColor)";
      input Real specularCoefficient = world.defaultSpecularCoefficient "Ignored: no animation";
      parameter StateSelect stateSelect = StateSelect.prefer;
      final parameter Real e[3] = Modelica.Math.Vectors.normalizeWithAssert(n) "Unit vector in direction of prismatic axis n (MSL: asserts length(n) > 0)";
      Real s(start = 0, final stateSelect = stateSelect) "Relative distance between frame_a and frame_b";
      Real v(start = 0, final stateSelect = stateSelect) "First derivative of s (relative velocity)";
      Real a(start = 0) "Second derivative of s (relative acceleration)";
      Real f "Actuation force in direction of joint axis";
    protected
      Real r[3] "Translation from the parent frame's origin to the child's, parent (= child) coordinates";
      Real S[6] "Motion subspace in child coordinates";
      Real IAc[6, 6];
      Real pAc[6];
      Real Vc[6];
      Real ap[6];
      Real c[6];
      Real U[6];
      Real D;
      Real u;
      Real Ia[6, 6];
      Real pa[6];
    equation
      Connections.branch(frame_a.R, frame_b.R);
      axis.s = s;
      support.s = 0;
      f = axis.f;
      v = der(s);
      a = der(v);
      if Connections.rooted(frame_a.R) then
        S = cat(1, zeros(3), e);
        r = e * s;
        frame_b.r_0 = frame_a.r_0 + transpose(frame_a.R.T) * r;
        frame_b.R.T = frame_a.R.T;
        frame_b.R.w = frame_a.R.w;
        frame_b.v = frame_a.v + cross(frame_a.R.w, r) + e * v;
        Vc = cat(1, frame_b.R.w, frame_b.v);
        IAc = -Math.unpackSym(frame_b.IA);
        pAc = -frame_b.pA;
        ap = cat(1, frame_a.a[1:3], frame_a.a[4:6] - cross(r, frame_a.a[1:3]));
        frame_b.a = ap + c + S * a;
        frame_b.dI = zeros(21);
        frame_a.IA = Math.translateInertia(Math.packSym(Ia), r);
        frame_a.pA = cat(1, pa[1:3] + cross(r, pa[4:6]), pa[4:6]);
        cat(1, frame_a.t, frame_a.f) = Math.unpackSym(frame_a.IA) * frame_a.a + frame_a.pA;
        frame_a.dv = zeros(3);
      else
        S = cat(1, zeros(3), -e);
        r = -e * s;
        frame_a.r_0 = frame_b.r_0 + transpose(frame_b.R.T) * r;
        frame_a.R.T = frame_b.R.T;
        frame_a.R.w = frame_b.R.w;
        frame_a.v = frame_b.v + cross(frame_b.R.w, r) - e * v;
        Vc = cat(1, frame_a.R.w, frame_a.v);
        IAc = -Math.unpackSym(frame_a.IA);
        pAc = -frame_a.pA;
        ap = cat(1, frame_b.a[1:3], frame_b.a[4:6] - cross(r, frame_b.a[1:3]));
        frame_a.a = ap + c + S * a;
        frame_a.dI = zeros(21);
        frame_b.IA = Math.translateInertia(Math.packSym(Ia), r);
        frame_b.pA = cat(1, pa[1:3] + cross(r, pa[4:6]), pa[4:6]);
        cat(1, frame_b.t, frame_b.f) = Math.unpackSym(frame_b.IA) * frame_b.a + frame_b.pA;
        frame_b.dv = zeros(3);
      end if;
      c = cat(1, zeros(3), cross(Vc[1:3], S[4:6] * v)) "crm(Vc) (S v) with S = {0, +-e}";
      U = IAc * S;
      D = S * U;
      u = f - S * pAc;
      Ia = IAc - {{U[i] * U[j] for j in 1:6} for i in 1:6} / D;
      pa = pAc + Ia * c + U * (u / D);
      a = (u - U * (ap + c)) / D;
    end Prismatic;
  end Joints;

  package Forces
    package Internal
      function standardGravityAcceleration "Uniform or point gravity field (MSL name and formula)"
        extends Interfaces.partialGravityAcceleration;
        input Types.GravityTypes gravityType "Type of gravity field";
        input Real g[3] "Constant gravity acceleration, world (UniformGravity)";
        input Real mu "Field constant (PointGravity)";
      algorithm
        gravity := if gravityType == Types.GravityTypes.UniformGravity then g
                   else if gravityType == Types.GravityTypes.PointGravity then -(mu / (r * r)) * (r / sqrt(r * r))
                   else zeros(3);
      end standardGravityAcceleration;
    end Internal;

    model WorldForce "External force acting at frame_b (MSL names; resolveInFrame world or frame_b)"
      Modelica.Blocks.Interfaces.RealInput force[3];
      Interfaces.Frame_b frame_b;
      parameter Types.ResolveInFrameB resolveInFrame = Types.ResolveInFrameB.world;
      parameter Boolean animation = true "Ignored: no animation";
    equation
      frame_b.f = -(if resolveInFrame == Types.ResolveInFrameB.world then frame_b.R.T * force else force);
      frame_b.t = zeros(3);
      frame_b.pA = cat(1, frame_b.t, frame_b.f);
      frame_b.IA = zeros(21);
      frame_b.dv = zeros(3);
    end WorldForce;

    model WorldForceAndTorque "External force and torque acting at frame_b (MSL names; resolveInFrame world or frame_b)"
      Modelica.Blocks.Interfaces.RealInput force[3];
      Modelica.Blocks.Interfaces.RealInput torque[3];
      Interfaces.Frame_b frame_b;
      parameter Types.ResolveInFrameB resolveInFrame = Types.ResolveInFrameB.world;
      parameter Boolean animation = true "Ignored: no animation";
    equation
      frame_b.f = -(if resolveInFrame == Types.ResolveInFrameB.world then frame_b.R.T * force else force);
      frame_b.t = -(if resolveInFrame == Types.ResolveInFrameB.world then frame_b.R.T * torque else torque);
      frame_b.pA = cat(1, frame_b.t, frame_b.f);
      frame_b.IA = zeros(21);
      frame_b.dv = zeros(3);
    end WorldForceAndTorque;

    model Spring "Linear translational spring with optional point mass on the connection line (MSL names)"
      // MSL Forces.Spring = Translational.Components.Spring inside LineForceWithMass (LineForceBase), as one force element.
      // Kinematics as LineForceBase: length = |r_rel_0|, s = max(length, s_small) only guards the direction e = r_rel_0/s,
      // and length > s_small is asserted. Spring law on the actual length: f = c (length - s_unstretched).
      // Point forces at both origins, no torques. MSL: frame_a.f = resolve2(R_a, m (1-h) ag_CM - e fa),
      // frame_b.f = resolve2(R_b, m h ag_CM - e fb), where MSL's flange connection gives fa = f, fb = -f, and
      // ag_CM = (1-h) a_a + h a_b - g(r_CM) (h = lengthFraction). For m = 0 MSL's massless branch applies: no CM kinematics
      // and no gravity evaluation. Written in this library's form: each frame's cut wrench is IA a + pA, with the frame's
      // own acceleration term as a point mass at its origin (IA) and the rest in pA.
      outer MultiBodyLie.World world;
      Interfaces.Frame_a frame_a;
      Interfaces.Frame_b frame_b;
      parameter Boolean animation = true "Ignored: no animation";
      parameter Boolean showMass = true "Ignored: no animation";
      parameter Real c(final min = 0) "Spring constant [N/m]";
      parameter Real s_unstretched = 0 "Unstretched spring length [m]";
      parameter Real m(min = 0) = 0 "Spring mass on the connection line [kg]";
      parameter Real lengthFraction(min = 0, max = 1) = 0.5 "Mass location from frame_a as a fraction of the distance a -> b";
      input Real width = world.defaultForceWidth "Ignored: no animation (width of spring)";
      input Real coilWidth = width / 10 "Ignored: no animation (width of spring coil)";
      parameter Integer numberOfWindings = 5 "Ignored: no animation";
      input Integer color[3] = {0, 0, 255} "Ignored: no animation (MSL Types.Defaults.SpringColor)";
      input Real specularCoefficient = world.defaultSpecularCoefficient "Ignored: no animation";
      input Real massDiameter = max(0, (width - 2 * coilWidth) * 0.9) "Ignored: no animation (diameter of mass point sphere)";
      input Integer massColor[3] = {0, 128, 255} "Ignored: no animation (MSL Types.Defaults.BodyColor)";
      parameter Real s_small = 1e-10 "Prevent zero-division if distance between frame_a and frame_b is zero";
      parameter Boolean fixedRotationAtFrame_a = false "Not supported: must be false";
      parameter Boolean fixedRotationAtFrame_b = false "Not supported: must be false";
      Real r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
      Real e_a[3] "Unit vector from frame_a to frame_b, resolved in frame_a";
      Real f "Line force (positive, if acting on frame_b and directed from frame_a to frame_b)";
      Real length "Distance between the origin of frame_a and the origin of frame_b";
      Real s "(Guarded) distance between the origins (>= s_small)";
      Real r_rel_0[3] "Position vector from frame_a to frame_b, world";
      Real e_rel_0[3] "Unit vector from frame_a to frame_b, world";
    protected
      final parameter Real ma = m * (1 - lengthFraction) "Line mass share carried by frame_a";
      final parameter Real mb = m * lengthFraction "Line mass share carried by frame_b";
      Real acc_a[3] "Origin acceleration of frame_a, world";
      Real acc_b[3] "Origin acceleration of frame_b, world";
      // Gravity in a binding, as in Body (an equation-section call read the World class default instead of the inner
      // modifier under rumoca b3afe4cb). Not evaluated at m = 0, as MSL's massless branch.
      Real g_CM[3] = if m > 0 then world.gravityAcceleration(frame_a.r_0 + lengthFraction * r_rel_0) else zeros(3) "Gravity at the line mass, world";
    equation
      assert(not fixedRotationAtFrame_a and not fixedRotationAtFrame_b, "Spring: fixedRotationAtFrame_a/b is not supported");
      assert(noEvent(length > s_small), "
The distance between the origin of frame_a and the origin of frame_b
of a line force component became smaller as parameter s_small
(= a small number, defined in the \"Advanced\" menu). The distance is
set to s_small, although it is smaller, to avoid a division by zero
when computing the direction of the line force. Possible reasons
for this situation:
- At initial time the distance may already be zero: Change the initial
  positions of the bodies connected by this element.
- Hardware stops are not modeled or are modeled not stiff enough.
  Include stops, e.g., stiff springs, or increase the stiffness
  if already present.
- Another error in your model may lead to unrealistically large forces
  and torques that would in reality destroy the stops.
- The flange_b connector might be defined by a pre-defined motion,
  e.g., with Modelica.Mechanics.Translational.Position and the
  predefined flange_b.s is zero or negative.
");
      r_rel_0 = frame_b.r_0 - frame_a.r_0;
      length = sqrt(r_rel_0 * r_rel_0);
      s = noEvent(max(length, s_small));
      e_rel_0 = r_rel_0 / s;
      r_rel_a = Frames.resolve2(frame_a.R, r_rel_0);
      e_a = r_rel_a / s;
      f = c * (length - s_unstretched);
      if m > 0 then
        acc_a = transpose(frame_a.R.T) * (frame_a.a[4:6] + cross(frame_a.R.w, frame_a.v));
        acc_b = transpose(frame_b.R.T) * (frame_b.a[4:6] + cross(frame_b.R.w, frame_b.v));
        // frame_a: m(1-h)((1-h) a_a + h a_b - g) - e f; frame_b: m h((1-h) a_a + h a_b - g) + e f.
        frame_a.IA = Math.packSym(Math.spatialInertia(ma * (1 - lengthFraction), zeros(3), zeros(3, 3)));
        frame_a.pA = cat(1, zeros(3), ma * (1 - lengthFraction) * cross(frame_a.R.w, frame_a.v)
          + frame_a.R.T * (ma * lengthFraction * acc_b - ma * g_CM - e_rel_0 * f));
        frame_b.IA = Math.packSym(Math.spatialInertia(mb * lengthFraction, zeros(3), zeros(3, 3)));
        frame_b.pA = cat(1, zeros(3), mb * lengthFraction * cross(frame_b.R.w, frame_b.v)
          + frame_b.R.T * (mb * (1 - lengthFraction) * acc_a - mb * g_CM + e_rel_0 * f));
      else
        // MSL massless branch: frame_a.f = -resolve2(R_a, e fa), frame_b.f = -resolve2(R_b, e fb).
        acc_a = zeros(3);
        acc_b = zeros(3);
        frame_a.IA = zeros(21);
        frame_a.pA = cat(1, zeros(3), -(frame_a.R.T * (e_rel_0 * f)));
        frame_b.IA = zeros(21);
        frame_b.pA = cat(1, zeros(3), frame_b.R.T * (e_rel_0 * f));
      end if;
      cat(1, frame_a.t, frame_a.f) = Math.unpackSym(frame_a.IA) * frame_a.a + frame_a.pA;
      cat(1, frame_b.t, frame_b.f) = Math.unpackSym(frame_b.IA) * frame_b.a + frame_b.pA;
      frame_a.dv = zeros(3);
      frame_b.dv = zeros(3);
    end Spring;
  end Forces;
end MultiBodyLie;
