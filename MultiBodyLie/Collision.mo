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
  // Shapes come from a closed set of kinds. maxPoints gives each pair of kinds its largest number of
  // contact points; a pair the table marks with zero is outside the library. Of the seven pairs with a
  // positive entry, the three plane pairs (plane against sphere, capsule and box) have their geometry
  // here, for a plane fixed in the world. The other four (sphere against sphere, capsule and box, and
  // capsule against capsule) have one point each: sphereSphere, sphereCapsule, sphereBox and
  // capsuleCapsule give their geometry (flipped exchanges the two shapes), shapePair chooses among
  // them by the two kinds, and pairWrenches turns a point between two bodies into a balanced pair of
  // wrenches. capsuleCapsule returns the nearest points of the two segments; the second point the
  // table allows, for capsules lying along each other, is not written. A pair without geometry is
  // refused by an assertion (in sceneWrench and in shapePair), never skipped.
  //
  // Where two centres coincide the line between them gives no normal. Two spheres then take the
  // world z axis by convention. A capsule has an axis, so a point on its centreline takes a normal
  // across that axis (perpendicular), and two capsules whose centrelines meet take the normal across
  // both; the witness points then lie on the surfaces, as everywhere else.
  //
  // The geometry functions are plain functions of the current poses. Branches inside a function create
  // no events, so geometry never stops the integrator. A contact becomes active through the clamp in
  // the force law, not through a discrete decision.
  //
  // One function, contactForce, owns the force law for every pair of kinds. It is the law of
  // MultiBodyLie.Contact.groundWrench written for an arbitrary unit normal. A contact point keeps its
  // witness points relative to the origins of the frames that own the two shapes, as groundWrench
  // does: that is what a wrench about the frame origin needs, and it does not lose digits far from
  // the world origin. With the ground plane (point {0, 0, 0}, normal {0, 0, 1}) the tests compare
  // planeSphere and contactForce with groundWrench for exact numerical equality on fixed poses and
  // velocities. That is a result for those inputs, not a promise for every input: the gap is summed here as
  // origin + (arm - radius) and in groundWrench's clear test as (origin + arm) - radius, so a point
  // within rounding of the clear margin can be cut off by one and not by the other.
  //
  // For plane contacts a shape is sampled by spheres: a sphere by itself, a capsule by the two spheres
  // at the ends of its segment, and a box by eight corner points of radius zero. The smallest signed
  // gap of the shape to the plane is the smallest gap of its samples, and each sample receives the full
  // point law independently. For a box this is corner sampling, not a model of pressure over a face.
  // planeSpheres owns the samples and planeSphere is the one geometry function of all the plane pairs.
  //
  // The material of a contact is pairMaterial of the materials of its two shapes: the mean of their
  // law parameters and the larger friction coefficient, so two shapes of one material have exactly
  // that material.
  //
  // There are two consumers. sceneWrench (model BodyShapes) is the shapes of a body against the planes
  // of the world. bodiesWrench (model BodyPair) is the shapes of one body against the shapes of
  // another; which pairs of bodies can touch is stated by the BodyPair instances of a model, one per
  // pair, and nothing is searched for.

  type ShapeKind = enumeration(
    Plane "Half-space below the plane through the shape origin with normal along the shape z axis",
    Sphere "Sphere about the shape origin; size = {radius, 0, 0}",
    Capsule "Segment along the shape x axis swept by a sphere; size = {radius, half length, 0}",
    Box "Box about the shape origin; size = half extents along the shape axes")
    "Closed set of collision shape kinds";

  constant Integer maxPoints[ShapeKind, ShapeKind] = {
    {0, 1, 2, 8},
    {1, 1, 1, 1},
    {2, 1, 2, 0},
    {8, 1, 0, 0}}
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

  function materialRow "One row of a scene's material table as a Material"
    input Integer row "The row";
    input Real k[:](each unit = "N/m") "Contact stiffness of each material";
    input Real d[size(k, 1)](each unit = "s/m") "Hunt-Crossley damping of each material";
    input Real mu[size(k, 1)] "Friction coefficient of each material";
    input Real eps[size(k, 1)](each unit = "m") "Penetration smoothing of each material";
    input Real v_eps[size(k, 1)](each unit = "m/s") "Friction regularization speed of each material";
    output Material material "The material of that row";
  algorithm
    material := Material(k = k[row], d = d[row], mu = mu[row], eps = eps[row], v_eps = v_eps[row]);
  end materialRow;

  function pairMaterial "Material of a contact between a shape of material a and a shape of material b"
    input Material a "Material of one shape";
    input Material b "Material of the other shape";
    output Material pair "Mean of the law parameters, larger friction coefficient";
  algorithm
    // The rule MuJoCo applies by default to two geometries of equal priority: parameters mixed with
    // equal weights, friction the larger of the two. (x + x) / 2 is x, so two shapes of one material
    // keep exactly that material.
    pair := Material(k = (a.k + b.k) / 2, d = (a.d + b.d) / 2, mu = max(a.mu, b.mu), eps = (a.eps + b.eps) / 2,
      v_eps = (a.v_eps + b.v_eps) / 2);
  end pairMaterial;

  function planeSphere "Contact point of a sphere (shape a) with a plane fixed in the world (shape b): one point"
    input Real planePoint[3](each unit = "m") "A point of the plane, world frame";
    input Real planeNormal[3] "Unit normal of the plane, world frame, pointing out of the half-space";
    input Real origin[3](each unit = "m") "Origin of the frame that owns the sphere, world frame";
    input Real arm[3](each unit = "m") "Sphere centre relative to that origin, resolved in the world frame";
    input Real radius(unit = "m") "Sphere radius";
    output ContactPoint point "The sphere's support point against the plane normal and its foot on the plane";
  protected
    Real armA[3](each unit = "m") "The sphere's support point (centre minus radius times the normal), relative to the origin";
    Real gap(unit = "m") "Its distance from the plane";
  algorithm
    armA := arm - radius * planeNormal;
    gap := (origin - planePoint) * planeNormal + armA * planeNormal;
    point := ContactPoint(gap = gap, n = planeNormal, armA = armA, armB = origin + armA - gap * planeNormal);
  end planeSphere;

  function planeSpheres "The sphere samples of a shape for the point-force model against a plane"
    input ShapeKind kind "Kind of the shape";
    input Real r[3](each unit = "m") "Shape origin, resolved in the owning frame";
    input Real T[3, 3] "Rotation from the owning frame to the shape frame";
    input Real size[3](each unit = "m") "Dimensions of the shape";
    output Integer count "Number of spheres: maxPoints[Plane, kind]";
    output Real centre[8, 3](each unit = "m") "Sphere centres, resolved in the owning frame; rows after count are zero";
    output Real radius(unit = "m") "Radius of every one of the spheres";
  algorithm
    centre := zeros(8, 3);
    if kind == ShapeKind.Sphere then
      count := 1;
      centre[1, :] := r;
      radius := size[1];
    elseif kind == ShapeKind.Capsule then
      count := 2;
      centre[1, :] := r - size[2] * T[1, :];
      centre[2, :] := r + size[2] * T[1, :];
      radius := size[1];
    elseif kind == ShapeKind.Box then
      count := 8;
      for x in 0:1 loop
        for y in 0:1 loop
          for z in 0:1 loop
            centre[1 + 4 * x + 2 * y + z, :] := r + transpose(T) * {(2 * x - 1) * size[1], (2 * y - 1) * size[2],
              (2 * z - 1) * size[3]};
          end for;
        end for;
      end for;
      radius := 0;
    else
      count := 0;
      radius := 0;
    end if;
  end planeSpheres;

  function flipped "The same contact point with the two shapes exchanged"
    input ContactPoint point "A contact point of a shape a with a shape b";
    output ContactPoint other "The point of b with a: same gap, opposite normal, witness points exchanged";
  algorithm
    other := ContactPoint(gap = point.gap, n = -point.n, armA = point.armB, armB = point.armA);
  end flipped;

  function perpendicular "A unit vector perpendicular to a unit vector"
    input Real axis[3] "A unit vector";
    output Real n[3] "A unit vector perpendicular to it";
  protected
    Real across[3] "The input crossed with the world axis it is least along";
  algorithm
    // Among equal components the lower world axis is taken. The cross product is at least sqrt(2 / 3) long.
    if abs(axis[1]) <= abs(axis[2]) and abs(axis[1]) <= abs(axis[3]) then
      across := cross(axis, {1, 0, 0});
    elseif abs(axis[2]) <= abs(axis[3]) then
      across := cross(axis, {0, 1, 0});
    else
      across := cross(axis, {0, 0, 1});
    end if;
    n := across / sqrt(across * across);
  end perpendicular;

  function centredSpheres "Contact point of two spheres, given the normal to use when their centres coincide"
    input Real originA[3](each unit = "m") "Origin of the frame that owns a, world frame";
    input Real centreA[3](each unit = "m") "Centre of a relative to that origin, resolved in the world frame";
    input Real radiusA(unit = "m") "Radius of a";
    input Real originB[3](each unit = "m") "Origin of the frame that owns b, world frame";
    input Real centreB[3](each unit = "m") "Centre of b relative to that origin, resolved in the world frame";
    input Real radiusB(unit = "m") "Radius of b";
    input Real coincident[3] "Unit normal from b towards a to use when the centres coincide";
    output ContactPoint point "The support points of the two spheres along the normal";
  protected
    Real d[3](each unit = "m") "From the centre of b to the centre of a";
    Real distance(unit = "m") "Distance between the centres";
    Real n[3] "Unit normal from b towards a";
  algorithm
    // The origins are subtracted from each other first, so the small offsets between two bodies are not
    // added to large positions. Digits the caller's inputs have already lost are not recovered.
    d := (originA - originB) + (centreA - centreB);
    distance := sqrt(d * d);
    n := if distance > 0 then d / distance else coincident;
    point := ContactPoint(gap = distance - radiusA - radiusB, n = n, armA = centreA - radiusA * n,
      armB = centreB + radiusB * n);
  end centredSpheres;

  function sphereSphere "Contact point of a sphere (shape a) with a sphere (shape b): one point"
    input Real originA[3](each unit = "m") "Origin of the frame that owns a, world frame";
    input Real centreA[3](each unit = "m") "Centre of a relative to that origin, resolved in the world frame";
    input Real radiusA(unit = "m") "Radius of a";
    input Real originB[3](each unit = "m") "Origin of the frame that owns b, world frame";
    input Real centreB[3](each unit = "m") "Centre of b relative to that origin, resolved in the world frame";
    input Real radiusB(unit = "m") "Radius of b";
    output ContactPoint point "The support points of the two spheres on the line of their centres";
  algorithm
    // Coincident centres have no line of centres; the normal is then the world z axis by convention.
    point := centredSpheres(originA, centreA, radiusA, originB, centreB, radiusB, {0, 0, 1});
  end sphereSphere;

  function sphereCapsule "Contact point of a sphere (shape a) with a capsule (shape b): one point"
    input Real originA[3](each unit = "m") "Origin of the frame that owns a, world frame";
    input Real centreA[3](each unit = "m") "Centre of a relative to that origin, resolved in the world frame";
    input Real radiusA(unit = "m") "Radius of a";
    input Real originB[3](each unit = "m") "Origin of the frame that owns b, world frame";
    input Real centreB[3](each unit = "m") "Middle of the capsule's segment relative to that origin, resolved in the world frame";
    input Real axisB[3] "Unit vector along the capsule's segment, world frame";
    input Real halfLengthB(unit = "m") "Half length of the segment";
    input Real radiusB(unit = "m") "Radius of the capsule";
    output ContactPoint point "The sphere against the capsule's sphere nearest to it";
  protected
    Real s(unit = "m") "Position on the segment of its point nearest the centre of a";
  algorithm
    s := min(max(((originA - originB) + (centreA - centreB)) * axisB, -halfLengthB), halfLengthB);
    // A sphere centred on the capsule's segment has no nearest side; the normal is then across the axis,
    // which keeps the capsule's witness point on its surface.
    point := centredSpheres(originA, centreA, radiusA, originB, centreB + s * axisB, radiusB, perpendicular(axisB));
  end sphereCapsule;

  function sphereBox "Contact point of a sphere (shape a) with a box (shape b): one point"
    input Real originA[3](each unit = "m") "Origin of the frame that owns a, world frame";
    input Real centreA[3](each unit = "m") "Centre of a relative to that origin, resolved in the world frame";
    input Real radiusA(unit = "m") "Radius of a";
    input Real originB[3](each unit = "m") "Origin of the frame that owns b, world frame";
    input Real centreB[3](each unit = "m") "Centre of the box relative to that origin, resolved in the world frame";
    input Real TB[3, 3] "Rotation from the world frame to the box frame";
    input Real halfB[3](each unit = "m") "Half extents of the box along its axes";
    output ContactPoint point "The sphere against the point of the box nearest its centre";
  protected
    Real p[3](each unit = "m") "Centre of a relative to the centre of the box, box frame";
    Real q[3](each unit = "m") "The point of the box nearest the centre of a, box frame";
    Real d[3](each unit = "m") "From that point to the centre of a, box frame";
    Real distance(unit = "m") "Its length";
    Real clear[3](each unit = "m") "Distance from the centre of a to the two faces across each axis, when it is inside";
    Real face[3] "Outward unit normal, box frame, of the face an inside centre is nearest to";
    Real n[3] "Unit normal from the box towards a, world frame";
  algorithm
    p := TB * ((originA - originB) + (centreA - centreB));
    q := {min(max(p[i], -halfB[i]), halfB[i]) for i in 1:3};
    d := p - q;
    distance := sqrt(d * d);
    if distance > 0 then
      // The centre is outside the box.
      n := transpose(TB) * (d / distance);
      point := ContactPoint(gap = distance - radiusA, n = n, armA = centreA - radiusA * n,
        armB = centreB + transpose(TB) * q);
    else
      // The centre is inside the box and leaves it through the nearest face; among equally near faces
      // the lower axis wins. The normal jumps where the nearest face changes, deep inside the box.
      clear := {halfB[i] - abs(p[i]) for i in 1:3};
      if clear[1] <= clear[2] and clear[1] <= clear[3] then
        face := {if p[1] < 0 then -1 else 1, 0, 0};
      elseif clear[2] <= clear[3] then
        face := {0, if p[2] < 0 then -1 else 1, 0};
      else
        face := {0, 0, if p[3] < 0 then -1 else 1};
      end if;
      n := transpose(TB) * face;
      point := ContactPoint(gap = -(abs(face) * clear) - radiusA, n = n, armA = centreA - radiusA * n,
        armB = centreB + transpose(TB) * (p + (abs(face) * clear) * face));
    end if;
  end sphereBox;

  function capsuleCapsule "Contact point of a capsule (shape a) with a capsule (shape b): the nearest points of their segments"
    input Real originA[3](each unit = "m") "Origin of the frame that owns a, world frame";
    input Real centreA[3](each unit = "m") "Middle of a's segment relative to that origin, resolved in the world frame";
    input Real axisA[3] "Unit vector along a's segment, world frame";
    input Real halfLengthA(unit = "m") "Half length of a's segment";
    input Real radiusA(unit = "m") "Radius of a";
    input Real originB[3](each unit = "m") "Origin of the frame that owns b, world frame";
    input Real centreB[3](each unit = "m") "Middle of b's segment relative to that origin, resolved in the world frame";
    input Real axisB[3] "Unit vector along b's segment, world frame";
    input Real halfLengthB(unit = "m") "Half length of b's segment";
    input Real radiusB(unit = "m") "Radius of b";
    output ContactPoint point "The spheres of the two capsules at the nearest points of their segments";
  protected
    Real r[3](each unit = "m") "From the middle of b's segment to the middle of a's";
    Real b "Cosine of the angle between the segments";
    Real c(unit = "m") "r along a's axis";
    Real f(unit = "m") "r along b's axis";
    Real across[3] "a's axis crossed with b's: perpendicular to both, as long as the sine of their angle";
    Real sine2 "Squared sine of the angle between the segments";
    Real s(unit = "m") "Position on a's segment of its nearest point";
    Real t(unit = "m") "Position on b's segment of its nearest point";
  algorithm
    r := (originA - originB) + (centreA - centreB);
    b := axisA * axisB;
    c := axisA * r;
    f := axisB * r;
    across := cross(axisA, axisB);
    sine2 := across * across;
    // The squared distance of the points at s and t is convex in (s, t) on the rectangle of the two
    // segments: take the free minimum along a, clamp it, take the best t for it, then the best s for
    // that t. The free minimum is (b f - c) / (1 - b b), written with cross products so that it keeps
    // its digits for segments at a small angle. Only segments whose axes have no cross product at all
    // are parallel: they have no free minimum and start from the middle of a, so of the nearest pairs
    // they have in common the one nearest that middle is returned.
    s := if sine2 > 0 then min(max(-(across * cross(r, axisB)) / sine2, -halfLengthA), halfLengthA) else 0;
    t := min(max(f + b * s, -halfLengthB), halfLengthB);
    s := min(max(b * t - c, -halfLengthA), halfLengthA);
    // Centrelines that meet have no nearest side; the normal is then across both axes (across one, for
    // parallel segments), which keeps both witness points on the surfaces.
    point := centredSpheres(originA, centreA + s * axisA, radiusA, originB, centreB + t * axisB, radiusB,
      if sine2 > 0 then across / sqrt(sine2) else perpendicular(axisA));
  end capsuleCapsule;

  function pairWrenches "The balanced wrenches of one contact point on the two frames that own its shapes"
    input ContactPoint point "A contact point of a shape a with a shape b";
    input Real vA[3](each unit = "m/s") "Velocity of the origin of the frame that owns a, world frame";
    input Real wA[3](each unit = "rad/s") "Angular velocity of that frame, resolved in the world frame";
    input Real vB[3](each unit = "m/s") "Velocity of the origin of the frame that owns b, world frame";
    input Real wB[3](each unit = "rad/s") "Angular velocity of that frame, resolved in the world frame";
    input Material material "Material of the pair";
    output Real fA[3](each unit = "N") "Force on a, world frame";
    output Real tauA[3](each unit = "N.m") "Torque on a about the origin of its frame, world frame";
    output Real fB[3](each unit = "N") "Force on b, world frame";
    output Real tauB[3](each unit = "N.m") "Torque on b about the origin of its frame, world frame";
  protected
    Real pA[3](each unit = "m") "The point both forces act at, relative to the origin of a's frame";
    Real pB[3](each unit = "m") "The same point relative to the origin of b's frame";
  algorithm
    // Both forces act at one point, midway between the witness points. Acting at the two witness
    // points instead would leave the moment of a friction force over the gap between them unbalanced.
    pA := point.armA - 0.5 * point.gap * point.n;
    pB := point.armB + 0.5 * point.gap * point.n;
    fA := contactForce(point, (vA + cross(wA, pA)) - (vB + cross(wB, pB)), material);
    tauA := cross(pA, fA);
    fB := -fA;
    tauB := cross(pB, fB);
  end pairWrenches;

  function shapePair "Contact point of a shape a with a shape b, each on its own frame, for the pairs of kinds that have one point"
    input ShapeKind kindA "Kind of a";
    input Real originA[3](each unit = "m") "Origin of the frame that owns a, world frame";
    input Real centreA[3](each unit = "m") "Origin of a relative to originA, resolved in the world frame";
    input Real TA[3, 3] "Rotation from the world frame to a's shape frame";
    input Real sizeA[3](each unit = "m") "Dimensions of a";
    input ShapeKind kindB "Kind of b";
    input Real originB[3](each unit = "m") "Origin of the frame that owns b, world frame";
    input Real centreB[3](each unit = "m") "Origin of b relative to originB, resolved in the world frame";
    input Real TB[3, 3] "Rotation from the world frame to b's shape frame";
    input Real sizeB[3](each unit = "m") "Dimensions of b";
    output ContactPoint point "The pair's contact point";
  algorithm
    // A capsule's segment lies along its shape x axis: the first row of its rotation.
    if kindA == ShapeKind.Sphere and kindB == ShapeKind.Sphere then
      point := sphereSphere(originA, centreA, sizeA[1], originB, centreB, sizeB[1]);
    elseif kindA == ShapeKind.Sphere and kindB == ShapeKind.Capsule then
      point := sphereCapsule(originA, centreA, sizeA[1], originB, centreB, TB[1, :], sizeB[2], sizeB[1]);
    elseif kindA == ShapeKind.Capsule and kindB == ShapeKind.Sphere then
      point := flipped(sphereCapsule(originB, centreB, sizeB[1], originA, centreA, TA[1, :], sizeA[2], sizeA[1]));
    elseif kindA == ShapeKind.Sphere and kindB == ShapeKind.Box then
      point := sphereBox(originA, centreA, sizeA[1], originB, centreB, TB, sizeB);
    elseif kindA == ShapeKind.Box and kindB == ShapeKind.Sphere then
      point := flipped(sphereBox(originB, centreB, sizeB[1], originA, centreA, TA, sizeA));
    elseif kindA == ShapeKind.Capsule and kindB == ShapeKind.Capsule then
      point := capsuleCapsule(originA, centreA, TA[1, :], sizeA[2], sizeA[1], originB, centreB, TB[1, :], sizeB[2],
        sizeB[1]);
    else
      assert(false,
        "Collision.shapePair supports sphere-sphere, sphere-capsule, sphere-box and capsule-capsule; other pairs of kinds are not implemented yet");
      // Not reached: the assertion above stops the evaluation. A function assigns its output on every path.
      point := ContactPoint(gap = 0, n = zeros(3), armA = zeros(3), armB = zeros(3));
    end if;
  end shapePair;

  function bodiesWrench "Balanced forces and torques on two bodies from the contacts between their shapes"
    input Real r0A[3](each unit = "m") "Origin of body a's frame, world frame";
    input Real TA[3, 3] "World-to-frame rotation of body a";
    input Real wA[3](each unit = "rad/s") "Angular velocity of body a, resolved in its frame";
    input Real v0A[3](each unit = "m/s") "Velocity of the origin of body a's frame, world frame";
    input ShapeKind kindA[:] "Kind of each shape of body a";
    input Real rA[size(kindA, 1), 3](each unit = "m") "Origin of each shape of a, body frame";
    input Real shapeTA[size(kindA, 1), 3, 3] "Rotation from a's body frame to each shape's frame";
    input Real sizeA[size(kindA, 1), 3](each unit = "m") "Dimensions of each shape of a";
    input Integer materialA[size(kindA, 1)] "Material row of each shape of a";
    input Integer groupA[size(kindA, 1)] "Collision group of each shape of a";
    input Real r0B[3](each unit = "m") "Origin of body b's frame, world frame";
    input Real TB[3, 3] "World-to-frame rotation of body b";
    input Real wB[3](each unit = "rad/s") "Angular velocity of body b, resolved in its frame";
    input Real v0B[3](each unit = "m/s") "Velocity of the origin of body b's frame, world frame";
    input ShapeKind kindB[:] "Kind of each shape of body b";
    input Real rB[size(kindB, 1), 3](each unit = "m") "Origin of each shape of b, body frame";
    input Real shapeTB[size(kindB, 1), 3, 3] "Rotation from b's body frame to each shape's frame";
    input Real sizeB[size(kindB, 1), 3](each unit = "m") "Dimensions of each shape of b";
    input Integer materialB[size(kindB, 1)] "Material row of each shape of b";
    input Integer groupB[size(kindB, 1)] "Collision group of each shape of b";
    input Real k[:](each unit = "N/m") "Contact stiffness of each material";
    input Real d[size(k, 1)](each unit = "s/m") "Hunt-Crossley damping of each material";
    input Real mu[size(k, 1)] "Friction coefficient of each material";
    input Real eps[size(k, 1)](each unit = "m") "Penetration smoothing of each material";
    input Real v_eps[size(k, 1)](each unit = "m/s") "Friction regularization speed of each material";
    input Boolean collides[:, :] "Groups that may touch";
    output Real fA[3](each unit = "N") "Force on body a, world frame";
    output Real tauA[3](each unit = "N.m") "Torque on body a about r0A, world frame";
    output Real fB[3](each unit = "N") "Force on body b, world frame: the opposite of fA";
    output Real tauB[3](each unit = "N.m") "Torque on body b about r0B, world frame";
  protected
    ContactPoint point "The contact point of the pair of shapes at hand";
    Real wAWorld[3](each unit = "rad/s") "Angular velocity of body a, world frame";
    Real wBWorld[3](each unit = "rad/s") "Angular velocity of body b, world frame";
    Real fi[3](each unit = "N") "Force of one contact point on a";
    Real taui[3](each unit = "N.m") "Its torque on a";
    Real fj[3](each unit = "N") "Force of the same point on b";
    Real tauj[3](each unit = "N.m") "Its torque on b";
  algorithm
    fA := zeros(3);
    tauA := zeros(3);
    fB := zeros(3);
    tauB := zeros(3);
    wAWorld := transpose(TA) * wA;
    wBWorld := transpose(TB) * wB;
    for i in 1:size(kindA, 1) loop
      for j in 1:size(kindB, 1) loop
        if collides[groupA[i], groupB[j]] then
          point := shapePair(kindA[i], r0A, transpose(TA) * rA[i, :], shapeTA[i, :, :] * TA, sizeA[i, :],
            kindB[j], r0B, transpose(TB) * rB[j, :], shapeTB[j, :, :] * TB, sizeB[j, :]);
          (fi, taui, fj, tauj) := pairWrenches(point, v0A, wAWorld, v0B, wBWorld,
            pairMaterial(materialRow(materialA[i], k, d, mu, eps, v_eps), materialRow(materialB[j], k, d, mu, eps, v_eps)));
          fA := fA + fi;
          tauA := tauA + taui;
          fB := fB + fj;
          tauB := tauB + tauj;
        end if;
      end for;
    end for;
  end bodiesWrench;

  function sceneWrench "Force and torque on a body from its shapes against the static shapes of a scene"
    input Real r0[3](each unit = "m") "Origin of the body frame, world frame";
    input Real T[3, 3] "World-to-body-frame rotation";
    input Real w[3](each unit = "rad/s") "Angular velocity of the body, resolved in the body frame";
    input Real v0[3](each unit = "m/s") "Velocity of the body frame origin, world frame";
    input ShapeKind kind[:] "Kind of each shape of the body";
    input Real r[size(kind, 1), 3](each unit = "m") "Origin of each shape, body frame";
    input Real shapeT[size(kind, 1), 3, 3] "Rotation from the body frame to each shape's frame";
    input Real size[size(kind, 1), 3](each unit = "m") "Dimensions of each shape";
    input Integer material[size(kind, 1)] "Material row of each shape";
    input Integer group[size(kind, 1)] "Collision group of each shape";
    input ShapeKind staticKind[:] "Kind of each static shape";
    input Real staticR[size(staticKind, 1), 3](each unit = "m") "Origin of each static shape, world frame";
    input Real staticT[size(staticKind, 1), 3, 3] "Rotation from the world frame to each static shape's frame";
    input Integer staticMaterial[size(staticKind, 1)] "Material row of each static shape";
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
    Material own "Material of the body's shape at hand";
    Integer count "Number of spheres that stand for the shape at hand";
    Real centre[8, 3](each unit = "m") "Their centres, body frame";
    Real radius(unit = "m") "Their radius";
    Real arm[3](each unit = "m") "A sphere centre relative to r0, resolved in the world frame";
    Real v[3](each unit = "m/s") "Velocity of the body's witness point, world frame";
    Real fi[3](each unit = "N") "Force of one contact point";
  algorithm
    f := zeros(3);
    tau := zeros(3);
    for i in 1:size(kind, 1) loop
      (count, centre, radius) := planeSpheres(kind[i], r[i, :], shapeT[i, :, :], size[i, :]);
      own := materialRow(material[i], k, d, mu, eps, v_eps);
      for j in 1:size(staticKind, 1) loop
        if collides[group[i], staticGroup[j]] then
          assert(count > 0 and staticKind[j] == ShapeKind.Plane,
            "Collision.sceneWrench supports a sphere, capsule or box of a body against a plane of the world; other pairs are not implemented yet");
          // Every shape has at most eight spheres; the rows after count are not contacts.
          for p in 1:8 loop
            if p <= count then
              arm := transpose(T) * centre[p, :];
              // The plane's normal is the z axis of its shape frame: the third row of its rotation.
              point := planeSphere(staticR[j, :], staticT[j, 3, :], r0, arm, radius);
              v := v0 + cross(transpose(T) * w, point.armA);
              fi := contactForce(point, v, pairMaterial(own, materialRow(staticMaterial[j], k, d, mu, eps, v_eps)));
              f := f + fi;
              tau := tau + cross(point.armA, fi);
            end if;
          end for;
        end if;
      end for;
    end for;
  end sceneWrench;

  model BodyShapes "Collision shapes of one rigid body against the static shapes of the scene: one force and torque at frame_a"
    parameter Integer n(min = 0) "Number of shapes";
    parameter ShapeKind kind[n] = fill(ShapeKind.Sphere, n) "Kind of each shape";
    parameter Real r[n, 3](each unit = "m") = zeros(n, 3) "Origin of each shape, resolved in frame_a";
    parameter Real T[n, 3, 3] = {identity(3) for i in 1:n} "Rotation from frame_a to each shape's frame";
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
    (f, tau) = sceneWrench(frame_a.r_0, frame_a.R.T, frame_a.R.w, v0, kind, r, T, size, material, group,
      scene.staticKind, scene.staticR, scene.staticT, scene.staticMaterial, scene.staticGroup,
      scene.k, scene.d, scene.mu, scene.eps, scene.v_eps, scene.collides);
    wrench.force = f;
    wrench.torque = tau;
  end BodyShapes;

  model BodyPair "Collision shapes of one rigid body against those of another: balanced forces and torques at frame_a and frame_b"
    parameter Integer nA(min = 0) "Number of shapes of the body at frame_a";
    parameter ShapeKind kindA[nA] = fill(ShapeKind.Sphere, nA) "Kind of each shape";
    parameter Real rA[nA, 3](each unit = "m") = zeros(nA, 3) "Origin of each shape, resolved in frame_a";
    parameter Real TA[nA, 3, 3] = {identity(3) for i in 1:nA} "Rotation from frame_a to each shape's frame";
    parameter Real sizeA[nA, 3](each unit = "m") = zeros(nA, 3) "Dimensions of each shape; their meaning is given by the kind";
    parameter Integer materialA[nA](each min = 1) = ones(nA) "Material row of each shape in the scene's table";
    parameter Integer groupA[nA](each min = 1) = ones(nA) "Collision group of each shape";
    parameter Integer nB(min = 0) "Number of shapes of the body at frame_b";
    parameter ShapeKind kindB[nB] = fill(ShapeKind.Sphere, nB) "Kind of each shape";
    parameter Real rB[nB, 3](each unit = "m") = zeros(nB, 3) "Origin of each shape, resolved in frame_b";
    parameter Real TB[nB, 3, 3] = {identity(3) for i in 1:nB} "Rotation from frame_b to each shape's frame";
    parameter Real sizeB[nB, 3](each unit = "m") = zeros(nB, 3) "Dimensions of each shape; their meaning is given by the kind";
    parameter Integer materialB[nB](each min = 1) = ones(nB) "Material row of each shape in the scene's table";
    parameter Integer groupB[nB](each min = 1) = ones(nB) "Collision group of each shape";
    input Real v0A[3](each unit = "m/s") "Velocity of the frame_a origin, world frame";
    input Real v0B[3](each unit = "m/s") "Velocity of the frame_b origin, world frame";
    outer Scene scene "The scene whose materials and groups the two bodies use";
    Interfaces.Frame_a frame_a;
    Interfaces.Frame_b frame_b;
    Forces.WorldForceAndTorque wrenchA(resolveInFrame = Types.ResolveInFrameB.world, animation = false);
    Forces.WorldForceAndTorque wrenchB(resolveInFrame = Types.ResolveInFrameB.world, animation = false);
    Real fA[3](each unit = "N") "Total contact force on the body at frame_a, world frame";
    Real tauA[3](each unit = "N.m") "Total contact torque on it about the frame_a origin, world frame";
    Real fB[3](each unit = "N") "Total contact force on the body at frame_b, world frame";
    Real tauB[3](each unit = "N.m") "Total contact torque on it about the frame_b origin, world frame";
  initial equation
    for i in 1:nA loop
      assert(materialA[i] <= scene.nMaterials, "A body shape names a material row the scene does not have");
      assert(groupA[i] <= scene.nGroups, "A body shape names a collision group the scene does not have");
    end for;
    for i in 1:nB loop
      assert(materialB[i] <= scene.nMaterials, "A body shape names a material row the scene does not have");
      assert(groupB[i] <= scene.nGroups, "A body shape names a collision group the scene does not have");
    end for;
  equation
    connect(frame_a, wrenchA.frame_b);
    connect(frame_b, wrenchB.frame_b);
    (fA, tauA, fB, tauB) = bodiesWrench(frame_a.r_0, frame_a.R.T, frame_a.R.w, v0A, kindA, rA, TA, sizeA, materialA, groupA,
      frame_b.r_0, frame_b.R.T, frame_b.R.w, v0B, kindB, rB, TB, sizeB, materialB, groupB,
      scene.k, scene.d, scene.mu, scene.eps, scene.v_eps, scene.collides);
    wrenchA.force = fA;
    wrenchA.torque = tauA;
    wrenchB.force = fB;
    wrenchB.torque = tauB;
  end BodyPair;
end Collision;
