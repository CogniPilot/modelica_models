within MultiBodyLie;
package Collision "Collision shapes, the scene tables and contact points between shapes"
  // Stage 1 of the collision library: the data a scene is made of and the boundary its consumers use.
  //
  // A scene is explicit tables of fixed size. A body lists its collision shapes, the world lists its
  // static shapes, the materials are rows of one table, and which groups may collide is one Boolean
  // matrix. Nothing registers itself while the model runs.
  //
  // A table of shapes is a set of arrays with one row per shape (kind, origin, rotation, size,
  // material, group), not an array of records: the tables are plain tensors all the way to the
  // functions that read them.
  //
  // Shapes come from a closed set of kinds. Every pair of kinds has one geometry function and a fixed
  // largest number of contact points (maxPoints); a pair the table marks with zero is not supported.
  //
  // The geometry functions are plain functions of the current poses. Branches inside a function create
  // no events, so geometry never stops the integrator. A contact becomes active through the clamp in
  // the force law, not through a discrete decision.
  //
  // One function, contactForce, owns the force law for every pair of kinds. It is the law of
  // MultiBodyLie.Contact.groundWrench written for an arbitrary unit normal. A contact point keeps its
  // witness points relative to the origins of the frames that own the two shapes, as groundWrench
  // does: that is what a wrench about the frame origin needs, and it does not lose digits far from
  // the world origin. With the ground plane (point {0, 0, 0}, normal {0, 0, 1}) planeSphere and
  // contactForce evaluate the same products and sums as groundWrench, in the same order.
  //
  // The material of a contact is the material of the moving body's shape. sceneWrench is the first
  // consumer: the spheres of a body against the planes of the world.

  type ShapeKind = enumeration(
    Plane "Half-space below the plane through the shape origin with normal along the shape z axis",
    Sphere "Sphere about the shape origin; size = {radius, 0, 0}",
    Capsule "Segment along the shape x axis swept by a sphere; size = {radius, half length, 0}",
    Box "Box about the shape origin; size = half extents along the shape axes")
    "Closed set of collision shape kinds";

  constant Integer maxPoints[ShapeKind, ShapeKind] = {
    {0, 1, 2, 4},
    {1, 1, 1, 1},
    {2, 1, 2, 0},
    {4, 1, 0, 0}}
    "Largest number of contact points of each pair of kinds; 0 marks a pair that is not supported";

  constant Real dampingSmoothing = 0.05
    "Smoothing width of the Hunt-Crossley damping factor (the value Contact.groundWrench uses)";

  constant Real clearMargin = 50
    "A point whose gap is at least this many smoothing lengths carries exactly no force (the margin Contact.groundWrench uses)";

  record Material "Parameters of the contact force law for one material"
    Real k(unit = "N/m") = 1e5 "Contact stiffness";
    Real d(unit = "s/m") = 0.5 "Hunt-Crossley damping";
    Real mu = 0.8 "Friction coefficient";
    Real eps(unit = "m") = 1e-4 "Penetration smoothing";
    Real v_eps(unit = "m/s") = 0.01 "Friction regularization speed";
  end Material;

  record ContactPoint "One contact point between a shape a and a shape b; a pair function returns its points in a fixed order, so a point's position is its feature"
    Real gap(unit = "m") "Distance between the two surfaces along the normal; negative when they overlap";
    Real n[3] "Unit normal, resolved in the world frame, pointing from shape b towards shape a";
    Real armA[3](each unit = "m")
      "Witness point on shape a relative to the origin of the frame that owns a, resolved in the world frame";
    Real armB[3](each unit = "m")
      "Witness point on shape b relative to the origin of the frame that owns b, resolved in the world frame";
  end ContactPoint;

  model Scene "The static shapes of the world, the materials and the groups that may collide"
    parameter Integer nStatic(min = 0) = 1 "Number of static shapes";
    parameter Integer nMaterials(min = 1) = 1 "Number of materials";
    parameter Integer nGroups(min = 1) = 1 "Number of collision groups";
    parameter ShapeKind staticKind[nStatic] = fill(ShapeKind.Plane, nStatic)
      "Kind of each static shape; the default scene is the ground plane z = 0 with normal +z";
    parameter Real staticR[nStatic, 3](each unit = "m") = zeros(nStatic, 3) "Origin of each static shape, world frame";
    parameter Real staticT[nStatic, 3, 3] = {identity(3) for i in 1:nStatic}
      "Rotation from the world frame to each static shape's frame";
    parameter Real staticSize[nStatic, 3](each unit = "m") = zeros(nStatic, 3)
      "Dimensions of each static shape; their meaning is given by the kind";
    parameter Integer staticMaterial[nStatic](each min = 1) = ones(nStatic) "Material row of each static shape";
    parameter Integer staticGroup[nStatic](each min = 1) = ones(nStatic) "Collision group of each static shape";
    parameter Real k[nMaterials](each unit = "N/m") = fill(1e5, nMaterials) "Contact stiffness of each material";
    parameter Real d[nMaterials](each unit = "s/m") = fill(0.5, nMaterials) "Hunt-Crossley damping of each material";
    parameter Real mu[nMaterials] = fill(0.8, nMaterials) "Friction coefficient of each material";
    parameter Real eps[nMaterials](each unit = "m") = fill(1e-4, nMaterials) "Penetration smoothing of each material";
    parameter Real v_eps[nMaterials](each unit = "m/s") = fill(0.01, nMaterials)
      "Friction regularization speed of each material";
    parameter Boolean collides[nGroups, nGroups] = fill(true, nGroups, nGroups)
      "collides[i, j] = true when a shape of group i may touch a shape of group j; must be symmetric";
  initial equation
    for i in 1:nStatic loop
      assert(staticMaterial[i] <= nMaterials, "A static shape names a material row the scene does not have");
      assert(staticGroup[i] <= nGroups, "A static shape names a collision group the scene does not have");
    end for;
    for i in 1:nGroups loop
      for j in 1:nGroups loop
        assert(collides[i, j] == collides[j, i], "The collides matrix of a scene must be symmetric");
      end for;
    end for;
  end Scene;

  function contactForce "Force on shape a at its witness point: Hunt-Crossley normal force and regularized Coulomb friction"
    input ContactPoint point "The contact point";
    input Real v[3](each unit = "m/s") "Velocity of shape a's witness point relative to shape b's, world frame";
    input Material material "Material of the pair";
    output Real f[3](each unit = "N") "Force on shape a, world frame; shape b receives its opposite";
  protected
    Real vn(unit = "m/s") "Speed along the normal, positive when the shapes separate";
    Real vt[3](each unit = "m/s") "Sliding velocity in the tangent plane";
    Real fn(unit = "N") "Normal force";
    Real speed(unit = "m/s") "Regularized sliding speed";
  algorithm
    if point.gap < clearMargin * material.eps then
      vn := v * point.n;
      vt := v - vn * point.n;
      fn := material.k * Contact.contactSoftplus(-point.gap, material.eps)
        * Contact.contactSoftplus(1 - material.d * vn, dampingSmoothing);
      speed := sqrt(vt * vt + material.v_eps ^ 2);
      f := fn * point.n - material.mu * fn * vt / speed;
    else
      f := zeros(3);
    end if;
  end contactForce;

  function planeSphere "Contact point of a sphere (shape a) with a plane fixed in the world (shape b): one point"
    input Real planePoint[3](each unit = "m") "A point of the plane, world frame";
    input Real planeNormal[3] "Unit normal of the plane, world frame, pointing out of the half-space";
    input Real origin[3](each unit = "m") "Origin of the frame that owns the sphere, world frame";
    input Real arm[3](each unit = "m") "Sphere centre relative to that origin, resolved in the world frame";
    input Real radius(unit = "m") "Sphere radius";
    output ContactPoint point "The point of the sphere nearest the plane and its foot on the plane";
  protected
    Real armA[3](each unit = "m") "The sphere's point nearest the plane, relative to the origin";
    Real gap(unit = "m") "Its distance from the plane";
  algorithm
    armA := arm - radius * planeNormal;
    gap := (origin - planePoint) * planeNormal + armA * planeNormal;
    point := ContactPoint(gap = gap, n = planeNormal, armA = armA, armB = origin + armA - gap * planeNormal);
  end planeSphere;

  function sceneWrench "Force and torque on a body from its shapes against the static shapes of a scene"
    input Real r0[3](each unit = "m") "Origin of the body frame, world frame";
    input Real T[3, 3] "World-to-body-frame rotation";
    input Real w[3](each unit = "rad/s") "Angular velocity of the body, resolved in the body frame";
    input Real v0[3](each unit = "m/s") "Velocity of the body frame origin, world frame";
    input ShapeKind kind[:] "Kind of each shape of the body";
    input Real r[size(kind, 1), 3](each unit = "m") "Origin of each shape, body frame";
    input Real size[size(kind, 1), 3](each unit = "m") "Dimensions of each shape";
    input Integer material[size(kind, 1)] "Material row of each shape";
    input Integer group[size(kind, 1)] "Collision group of each shape";
    input ShapeKind staticKind[:] "Kind of each static shape";
    input Real staticR[size(staticKind, 1), 3](each unit = "m") "Origin of each static shape, world frame";
    input Real staticT[size(staticKind, 1), 3, 3] "Rotation from the world frame to each static shape's frame";
    input Integer staticGroup[size(staticKind, 1)] "Collision group of each static shape";
    input Real k[:](each unit = "N/m") "Contact stiffness of each material";
    input Real d[size(k, 1)](each unit = "s/m") "Hunt-Crossley damping of each material";
    input Real mu[size(k, 1)] "Friction coefficient of each material";
    input Real eps[size(k, 1)](each unit = "m") "Penetration smoothing of each material";
    input Real v_eps[size(k, 1)](each unit = "m/s") "Friction regularization speed of each material";
    input Boolean collides[:, :] "Groups that may touch";
    output Real f[3](each unit = "N") "Force on the body, world frame";
    output Real tau[3](each unit = "N.m") "Torque on the body about r0, world frame";
  protected
    ContactPoint point "One contact point of the pair at hand";
    Material pair "Material of the pair at hand";
    Real arm[3](each unit = "m") "Shape origin relative to r0, resolved in the world frame";
    Real v[3](each unit = "m/s") "Velocity of the body's witness point, world frame";
    Real fi[3](each unit = "N") "Force of one contact point";
  algorithm
    f := zeros(3);
    tau := zeros(3);
    for i in 1:size(kind, 1) loop
      arm := transpose(T) * r[i, :];
      pair := Material(k = k[material[i]], d = d[material[i]], mu = mu[material[i]], eps = eps[material[i]],
        v_eps = v_eps[material[i]]);
      for j in 1:size(staticKind, 1) loop
        if collides[group[i], staticGroup[j]] then
          assert(kind[i] == ShapeKind.Sphere and staticKind[j] == ShapeKind.Plane,
            "Collision.sceneWrench supports a sphere of a body against a plane of the world; other pairs are not implemented yet");
          // The plane's normal is the z axis of its shape frame: the third row of its rotation.
          point := planeSphere(staticR[j, :], staticT[j, 3, :], r0, arm, size[i, 1]);
          v := v0 + cross(transpose(T) * w, point.armA);
          fi := contactForce(point, v, pair);
          f := f + fi;
          tau := tau + cross(point.armA, fi);
        end if;
      end for;
    end for;
  end sceneWrench;

  model BodyShapes "Collision shapes of one rigid body against the static shapes of the scene: one force and torque at frame_a"
    parameter Integer n(min = 0) "Number of shapes";
    parameter ShapeKind kind[n] = fill(ShapeKind.Sphere, n) "Kind of each shape";
    parameter Real r[n, 3](each unit = "m") = zeros(n, 3) "Origin of each shape, resolved in frame_a";
    parameter Real size[n, 3](each unit = "m") = zeros(n, 3) "Dimensions of each shape; their meaning is given by the kind";
    parameter Integer material[n](each min = 1) = ones(n) "Material row of each shape in the scene's table";
    parameter Integer group[n](each min = 1) = ones(n) "Collision group of each shape";
    input Real v0[3](each unit = "m/s") "Velocity of the frame_a origin, world frame";
    outer Scene scene "The scene whose static shapes, materials and groups this body collides with";
    Interfaces.Frame_a frame_a;
    Forces.WorldForceAndTorque wrench(resolveInFrame = Types.ResolveInFrameB.world, animation = false);
    Real f[3](each unit = "N") "Total contact force, world frame";
    Real tau[3](each unit = "N.m") "Total contact torque about the frame_a origin, world frame";
  initial equation
    for i in 1:n loop
      assert(material[i] <= scene.nMaterials, "A body shape names a material row the scene does not have");
      assert(group[i] <= scene.nGroups, "A body shape names a collision group the scene does not have");
    end for;
  equation
    connect(frame_a, wrench.frame_b);
    (f, tau) = sceneWrench(frame_a.r_0, frame_a.R.T, frame_a.R.w, v0, kind, r, size, material, group,
      scene.staticKind, scene.staticR, scene.staticT, scene.staticGroup,
      scene.k, scene.d, scene.mu, scene.eps, scene.v_eps, scene.collides);
    wrench.force = f;
    wrench.torque = tau;
  end BodyShapes;
end Collision;
