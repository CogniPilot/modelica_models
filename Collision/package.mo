within;
package Collision "Geometric queries between rigid shapes"
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

  record ContactPoint "One contact point between a shape a and a shape b; a pair function returns its points in a fixed order, so a point's position is its feature"
    Real gap(unit = "m") "Distance between the two surfaces along the normal; negative when they overlap";
    Real n[3] "Unit normal, resolved in the world frame, pointing from shape b towards shape a";
    Real armA[3](each unit = "m")
      "Witness point on shape a relative to the origin of the frame that owns a, resolved in the world frame";
    Real armB[3](each unit = "m")
      "Witness point on shape b relative to the origin of the frame that owns b, resolved in the world frame";
  end ContactPoint;

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
    Real distance(unit = "m") "Signed distance from the sphere centre to the box surface";
    Real clear[3](each unit = "m") "Signed inward clearance to the nearer face on each axis";
    Real deltaScale(unit = "m") "Largest outside offset at an edge or corner";
    Real scaled[3] "Outside offset scaled before taking its norm";
    Real lengthScaled;
    Integer axis "Nearest face axis; the lower index wins ties";
    Real side "Sign of the selected outward face normal";
    Real n[3] "Unit normal from the box towards a, world frame";
  algorithm
    p := TB * ((originA - originB) + (centreA - centreB));
    clear := {halfB[i] - abs(p[i]) for i in 1:3};
    if (clear[1] < 0 and clear[2] < 0) or (clear[1] < 0 and clear[3] < 0) or
        (clear[2] < 0 and clear[3] < 0) then
      // The nearest feature is an edge or corner.
      q := {min(max(p[i], -halfB[i]), halfB[i]) for i in 1:3};
      d := p - q;
      deltaScale := max(abs(d));
      if deltaScale > sqrt(Modelica.Constants.small) and deltaScale < sqrt(Modelica.Constants.inf/3) then
        distance := sqrt(d * d);
        n := transpose(TB) * (d / distance);
      else
        // Rescale extremes so squaring a finite nonzero offset cannot lose the normal.
        scaled := d / deltaScale;
        lengthScaled := sqrt(scaled * scaled);
        distance := deltaScale * lengthScaled;
        n := transpose(TB) * (scaled / lengthScaled);
      end if;
    else
      // Inside the box or outside a single face, the signed distance and normal
      // follow directly from the face plane, including a centre on the surface.
      axis := if clear[1] <= clear[2] and clear[1] <= clear[3] then 1
        elseif clear[2] <= clear[3] then 2 else 3;
      side := if p[axis] < 0 then -1 else 1;
      distance := -clear[axis];
      n := side * TB[axis, :];
      q := p;
      q[axis] := side * halfB[axis];
    end if;
    point := ContactPoint(gap = distance - radiusA, n = n, armA = centreA - radiusA * n,
      armB = centreB + transpose(TB) * q);
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
    // that t. The free minimum is (b f - c) / (1 - b b), written with cross products, which do not
    // cancel at a small angle. Segments count as parallel only when the computed sine2 is zero: the
    // axes are parallel, or their angle is so small (below about 1e-162) that its square underflows.
    // Those have no free minimum and start from the middle of a, so of the nearest pairs they have in
    // common the one nearest that middle is returned. No bound on the rounding error of the returned
    // pair at very small angles is claimed.
    s := if sine2 > 0 then min(max(-(across * cross(r, axisB)) / sine2, -halfLengthA), halfLengthA) else 0;
    t := min(max(f + b * s, -halfLengthB), halfLengthB);
    s := min(max(b * t - c, -halfLengthA), halfLengthA);
    // Centrelines that meet have no nearest side; the normal is then across both axes (across one, for
    // parallel segments), which keeps both witness points on the surfaces.
    point := centredSpheres(originA, centreA + s * axisA, radiusA, originB, centreB + t * axisB, radiusB,
      if sine2 > 0 then across / sqrt(sine2) else perpendicular(axisA));
  end capsuleCapsule;

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

  function boxBoxSupport "Separating-axis gap and support witness for two oriented boxes in a common frame"
    input Real center[3],half[3],axes[3,3],plate[3],extent[3],plateAxes[3,3];
    input Real contactMargin(unit="m")=.003 "Return early if an axis separates the boxes by more than this distance";
    output Real hit[7] "Axis gap, B-to-A normal, and witness on A; the witness is valid within the contact margin";
  protected
    Boolean better;
    Real delta[3],axis[3],normal[3],length,signedDistance,orientation,gap,p[3],onPlate[3],projection;
  algorithm
    delta:=center-plate;
    hit:={-1e5,0,0,0,0,0,0};
    for k in 1:15 loop
      if k<=3 then
        axis:=plateAxes[k,:];
      elseif k<=6 then
        axis:=axes[k-3,:];
      else
        axis:=cross(axes[1+div(k-7,3),:],plateAxes[1+mod(k-7,3),:]);
      end if;
      length:=sqrt(axis*axis);
      normal:=axis/max(length,1e-8);
      signedDistance:=normal*delta;
      gap:=abs(signedDistance)-abs(axes*normal)*half-abs(plateAxes*normal)*extent;
      better:=length>1e-8 and gap>hit[1];
      hit[1]:=if better then gap else hit[1];
      orientation:=if signedDistance>=0 then 1.0 else -1.0;
      for component in 1:3 loop
        hit[component+1]:=if better then orientation*normal[component] else hit[component+1];
      end for;
      if better and gap>contactMargin then return; end if;
    end for;
    // On a supporting face, use the closest location to the obstacle. This
    // keeps the lever arm on the struck part when a narrow rail hits a wall.
    onPlate:=plate;
    for i in 1:3 loop
      onPlate:=onPlate+plateAxes[i,:]*min(extent[i],max(-extent[i],plateAxes[i,:]*delta));
    end for;
    normal:=hit[2:4];
    p:=center;
    for i in 1:3 loop
      projection:=axes[i,:]*normal;
      if abs(projection)>1e-7 then
        p:=p-sign(projection)*half[i]*axes[i,:];
      else
        p:=p+min(half[i],max(-half[i],axes[i,:]*(onPlate-center)))*axes[i,:];
      end if;
    end for;
    hit[5:7]:=p;
    annotation(Documentation(info="<html><p>The overlap test uses the 15 oriented-box separating axes described in section 5 of Gottschalk, Lin, and Manocha (1996), <a href=\"https://doi.org/10.1145/237170.237244\">OBBTree</a>. The reported gap is a projection on a tested axis. The query can return on the first axis that exceeds the margin.</p></html>"));
  end boxBoxSupport;

  annotation(Documentation(info="<html><p>Computes signed gaps, normals and witness points from shape geometry in a common Cartesian frame. The package can be loaded independently of any dynamics library.</p><p>For contact materials and forces on multibody frames, see <a href=\"modelica://MultiBodyLie.Collision\">MultiBodyLie.Collision</a>.</p></html>"));
end Collision;
