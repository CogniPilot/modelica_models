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

  function orientedBoxSupport "Separating-axis gap and support witness for two oriented boxes in a common frame"
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
  end orientedBoxSupport;


  function triangleConvex "Contact between a triangle and outward-facing planes of a closed convex body"
    input Real tri[3,3](each unit="m") "Triangle vertices relative to shape A, resolved in the common frame";
    input Real face[:,4] "Plane rows {nx, ny, nz, offset}, with unit outward normals";
    input Real originB[3](each unit="m") "Origin of shape B relative to shape A in the common frame";
    output ContactPoint contact "Deepest face witness with negative gap, or zero gap and normal for a miss";
  protected
    Real polygon[size(face,1)+3,3],clipped[size(face,1)+3,3];
    Real a[3],b[3],n[3],pa,pb,point[3],depth,best;
    Integer count,newCount;
    Boolean allInside,allOutside;
  algorithm
    contact:=ContactPoint(0,zeros(3),zeros(3),zeros(3));
    // Reject a triangle on its original vertices before allocating work to
    // clip the polygon against the earlier faces of the convex body.
    for f in 1:size(face,1) loop
      n:=face[f,1:3];
      if tri[1,:]*n-face[f,4]>0 and tri[2,:]*n-face[f,4]>0 and tri[3,:]*n-face[f,4]>0 then return;end if;
    end for;
    polygon:=zeros(size(face,1)+3,3);clipped:=polygon;
    count:=3;polygon[1:3,:]:=tri;
    for f in 1:size(face,1) loop
      n:=face[f,1:3];newCount:=0;
      allInside:=true;allOutside:=true;
      for i in 1:count loop
        pb:=polygon[i,:]*n-face[f,4];
        allInside:=allInside and pb<=0;allOutside:=allOutside and pb>0;
      end for;
      if allOutside then return;end if;
      if not allInside then
        a:=polygon[count,:];pa:=a*n-face[f,4];
        for i in 1:count loop
          b:=polygon[i,:];pb:=b*n-face[f,4];
          if (pa<0 and pb>0) or (pa>0 and pb<0) then
            newCount:=newCount+1;
            clipped[newCount,:]:=a+(b-a)*(pa/(pa-pb));
          end if;
          if pb<=0 then
            newCount:=newCount+1;clipped[newCount,:]:=b;
          end if;
          a:=b;pa:=pb;
        end for;
        if newCount<3 then return;end if;
        count:=newCount;
        for i in 1:count loop polygon[i,:]:=clipped[i,:];end for;
      end if;
    end for;
    point:=zeros(3);
    for i in 1:count loop point:=point+polygon[i,:];end for;
    point:=point/count;
    best:=1e6;n:=zeros(3);
    for f in 1:size(face,1) loop
      depth:=face[f,4]-face[f,1:3]*point;
      if depth<best then best:=depth;n:=face[f,1:3];end if;
    end for;
    if best>0 then
      contact:=ContactPoint(-best,n,point,point+best*n-originB);
    end if;
    annotation(Inline=false,Documentation(info="<html><p>Clips the triangle successively against the outer planes using the method in Sutherland and Hodgman (1974), <a href=\"https://doi.org/10.1145/360767.360802\">Reentrant Polygon Clipping</a>. The centroid of the surviving polygon is the witness on the triangle.</p></html>"));
  end triangleConvex;
  function sphereBoundsOverlap "Cheap inclusive test for two enclosing spheres"
    input Real centreA[3](each unit="m"),radiusA(unit="m");
    input Real centreB[3](each unit="m"),radiusB(unit="m");
    output Boolean overlap;
  protected
    Real delta[3];
  algorithm
    delta:=centreA-centreB;
    overlap:=delta*delta<=(radiusA+radiusB)^2;
    annotation(Documentation(info="<html><p>Compares squared centre distance with the squared sum of the radii. Equality is a candidate. See Ericson, <a href=\"https://realtimecollisiondetection.net/books/rtcd/toc/index.html\">Real-Time Collision Detection</a>, section 4.3.</p></html>"));
  end sphereBoundsOverlap;

  function aabbOverlap "Inclusive intersection test for two closed axis-aligned boxes"
    input Real lowerA[3](each unit="m"),upperA[3](each unit="m");
    input Real lowerB[3](each unit="m"),upperB[3](each unit="m");
    output Boolean overlap;
  algorithm
    overlap:=true;
    for axis in 1:3 loop
      if lowerA[axis]>upperB[axis] or lowerB[axis]>upperA[axis] then
        overlap:=false;
        return;
      end if;
    end for;
    annotation(Documentation(info="<html><p>Tests for overlap along each of the three coordinate axes. See Ericson, <a href=\"https://realtimecollisiondetection.net/books/rtcd/toc/index.html\">Real-Time Collision Detection</a>, section 4.2.</p></html>"));
  end aabbOverlap;

  function boundsProjectionMin "Minimum projection of a local box onto a supplied axis"
    input Real lower[3](each unit="m"),upper[3](each unit="m");
    input Real axis[3];
    output Real minimum(unit="m");
  algorithm
    minimum:=sum({min(lower[i]*axis[i],upper[i]*axis[i]) for i in 1:3});
  end boundsProjectionMin;

  function pointConvexPlanes "Signed plane gap for a point against an affine convex body"
    input Real delta[3](each unit="m") "Point relative to the convex body's origin, in the common frame";
    input Real worldToBody[3,3] "Affine linear part mapping the query into the hull frame";
    input Real planes[:,4] "Outward plane normals and offsets in the hull frame";
    input Real limit(unit="m")=0 "A plane reaching this signed gap rejects the candidate";
    output ContactPoint point;
  protected
    Real local[3],gradient[3],length,gap,bestGap,normal[3];
    Integer face;
    Boolean rejected;
  algorithm
    bestGap:=-1e6;
    normal:={0,0,1};
    local:=worldToBody*delta;
    face:=1;
    rejected:=false;
    while face<=size(planes,1) and not rejected loop
      gradient:=transpose(worldToBody)*planes[face,1:3];
      length:=sqrt(gradient*gradient);
      gap:=(planes[face,1:3]*local-planes[face,4])/length;
      if gap>bestGap then
        bestGap:=gap;
        normal:=gradient/length;
      end if;
      rejected:=bestGap>=limit;
      face:=face+1;
    end while;
    point:=ContactPoint(bestGap,normal,zeros(3),delta-bestGap*normal);
    annotation(Documentation(info="<html><p>Projects onto each oriented face plane. The transpose of the world-to-body map converts the plane gradient to the common frame before normalization. A gap below the limit is the maximum over all faces. A gap at or above the limit certifies rejection. See Ericson, <a href=\"https://realtimecollisiondetection.net/books/rtcd/toc/index.html\">Real-Time Collision Detection</a>, sections 3.6 and 5.1.</p></html>"));
  end pointConvexPlanes;

  function vertexAxisGap "Signed interval gap along a separating axis"
    input Real a[8,3],b[8,3],axis[3];
    output Real test[4];
  protected
    Real norm2,n[3],p[8],q[8],forward,backward;
  algorithm
    norm2:=axis*axis;
    test:={-1e3,0,0,0};
    if norm2>1e-16 then
      n:=axis/sqrt(norm2);
      p:=a*n;
      q:=b*n;
      forward:=min(p)-max(q);
      backward:=min(q)-max(p);
      test:=if forward>=backward then cat(1,{forward},n) else cat(1,{backward},-n);
    end if;
  end vertexAxisGap;

  function hexahedronGap "Separating axis test for convex hexahedra with paired faces"
    input Real a[8,3](each unit="m"),b[8,3](each unit="m");
    input Integer faces[12,3];
    output Real best[4] "Signed axis gap in metres, then the three normal components";
  protected
    constant Integer edges[12,2]=[1,2;1,3;1,5;2,4;2,6;3,4;3,7;4,8;5,6;5,7;6,8;7,8];
    Real test[4],edge[3],vertices[8,3];
    Integer k;
  algorithm
    best:={-1e3,0,0,0};
    for body in 1:2 loop
      vertices:=if body==1 then a else b;
      // The two triangles of each face share one separating plane.
      for i in 1:6 loop
        k:=2*i-1;
        test:=vertexAxisGap(a,b,cross(vertices[faces[k,2],:]-vertices[faces[k,1],:],
          vertices[faces[k,3],:]-vertices[faces[k,1],:]));
        if test[1]>best[1] then
          best:=test;
        end if;
        if best[1]>=0 then
          return;
        end if;
      end for;
    end for;
    for i in 1:12 loop
      edge:=a[edges[i,2],:]-a[edges[i,1],:];
      for j in 1:12 loop
        test:=vertexAxisGap(a,b,cross(edge,b[edges[j,2],:]-b[edges[j,1],:]));
        if test[1]>best[1] then
          best:=test;
        end if;
        if best[1]>=0 then
          return;
        end if;
      end for;
    end for;
    annotation(Documentation(info="<html><p>Tests face normals and crosses of pairs of edges. The eight vertices use box-corner order, with six consecutive pairs of face triangles. This uses the separating-axis construction described for oriented boxes in Gottschalk, Lin and Manocha (1996), <a href=\"https://doi.org/10.1145/237170.237244\">OBBTree</a>, section 5. The vertex-projection variant also admits convex hexahedra whose quad faces are planar.</p></html>"));
  end hexahedronGap;

  function meanSupportPoint "Average of vertices on the supporting face"
    input Real vertices[8,3],direction[3];
    output Real point[3];
  protected
    Real projection[8],support;
    Integer count;
  algorithm
    projection:=vertices*direction;
    support:=max(projection);
    point:=zeros(3);
    count:=0;
    for i in 1:8 loop
      if support-projection[i]<1e-6 then
        point:=point+vertices[i,:];
        count:=count+1;
      end if;
    end for;
    point:=point/count;
  end meanSupportPoint;

  function closestSegmentPoint "Closest point on a finite line segment"
    input Real p[3],a[3],b[3];
    output Real closest[3];
  protected Real edge[3],u;
  algorithm
    edge:=b-a;
    u:=min(1,max(0,(p-a)*edge/max(1e-12,edge*edge)));
    closest:=a+u*edge;
  end closestSegmentPoint;

  function insideTriangleMesh "Ray parity through a watertight triangle mesh"
    input Real p[3],vertices[:,3];
    input Integer faces[:,3];
    output Boolean inside;
  protected
    Real a[3],b[3],c[3],y,z,det,u,v;
  algorithm
    inside:=false;
    y:=p[2]+1e-8;z:=p[3]+1.4142e-8;
    for i in 1:size(faces,1) loop
      a:=vertices[faces[i,1],:];b:=vertices[faces[i,2],:];c:=vertices[faces[i,3],:];
      if max(a[1],max(b[1],c[1]))>p[1] and y>=min(a[2],min(b[2],c[2])) and y<=max(a[2],max(b[2],c[2]))
        and z>=min(a[3],min(b[3],c[3])) and z<=max(a[3],max(b[3],c[3])) then
        b:=b-a;c:=c-a;
        det:=b[2]*c[3]-b[3]*c[2];
        if abs(det)>1e-12 then
          u:=((y-a[2])*c[3]-(z-a[3])*c[2])/det;
          v:=(b[2]*(z-a[3])-b[3]*(y-a[2]))/det;
          if u>=0 and v>=0 and u+v<1 and a[1]+u*b[1]+v*c[1]>p[1] then
            inside:=not inside;
          end if;
        end if;
      end if;
    end for;
    annotation(Documentation(info="<html><p>Counts parity along a fixed positive-x ray. A small transverse offset avoids exact edge hits. Triangle intersections use projected barycentric coordinates. For the underlying ray-triangle query see Ericson, <a href=\"https://realtimecollisiondetection.net/books/rtcd/toc/index.html\">Real-Time Collision Detection</a>, section 5.3.</p></html>"));
  end insideTriangleMesh;

  function triangleMeshGap "Signed distance and normal at the nearest triangle"
    input Real p[3],vertices[:,3];
    input Integer faces[:,3];
    input Boolean twoSided=false;
    input Integer firstFace=1,lastFace=size(faces,1),firstCap=size(faces,1)+1;
    output Real result[4];
  protected
    Real a[3],b[3],c[3],n[3],q[3],edge[3],delta[3],gap,best,d2,sgn,normal[3];
  algorithm
    best:=1e6;sgn:=1;normal:={0,0,1};
    for i in 1:size(faces,1) loop
      if (i>=firstFace and i<=lastFace) or i>=firstCap then
        a:=vertices[faces[i,1],:];b:=vertices[faces[i,2],:];c:=vertices[faces[i,3],:];
        gap:=max(0,max(min(a[1],min(b[1],c[1]))-p[1],p[1]-max(a[1],max(b[1],c[1]))));
        if gap*gap<best then
          n:=cross(b-a,c-a);
          n:=n/sqrt(max(1e-12,n*n));
          gap:=(p-a)*n;
          q:=p-gap*n;
          if cross(b-a,q-a)*n<0 or cross(c-b,q-b)*n<0 or cross(a-c,q-c)*n<0 then
            q:=closestSegmentPoint(p,a,b);
            edge:=closestSegmentPoint(p,b,c);
            if (p-edge)*(p-edge)<(p-q)*(p-q) then q:=edge;end if;
            edge:=closestSegmentPoint(p,c,a);
            if (p-edge)*(p-edge)<(p-q)*(p-q) then q:=edge;end if;
          end if;
          delta:=p-q;d2:=delta*delta;
          if d2<best then
            best:=d2;sgn:=if twoSided or gap>=0 then 1 else -1;
            normal:=if d2>1e-10 then sgn*delta/sqrt(d2) else n;
          end if;
        end if;
      end if;
    end for;
    result:=cat(1,{sgn*sqrt(best)},normal);
    annotation(Documentation(info="<html><p>Projects the point onto each candidate triangle plane and tests its three segments if the projection lies outside the face. Ericson describes the primitive closest-point problems in <a href=\"https://realtimecollisiondetection.net/books/rtcd/toc/index.html\">Real-Time Collision Detection</a>, sections 5.1.2 and 5.1.5. The signed result uses the nearest face side. Use <a href=\"modelica://Collision.insideTriangleMesh\">insideTriangleMesh</a> for closed-shell membership.</p></html>"));
  end triangleMeshGap;

  annotation(Documentation(info="<html><p>Computes signed gaps, normals and witness points from shape geometry in a common Cartesian frame. The package can be loaded independently of any dynamics library.</p><h4>Algorithm references</h4><ul><li>Candidate selection uses <a href=\"modelica://Collision.sphereBoundsOverlap\">bounding spheres</a> and <a href=\"modelica://Collision.aabbOverlap\">axis-aligned boxes</a>.</li><li>Convex witnesses use <a href=\"modelica://Collision.orientedBoxSupport\">oriented-box separating axes</a>, <a href=\"modelica://Collision.hexahedronGap\">hexahedron separating axes</a>, and <a href=\"modelica://Collision.triangleConvex\">Sutherland-Hodgman clipping</a>.</li><li>Mesh queries use <a href=\"modelica://Collision.triangleMeshGap\">nearest-triangle projection</a> and <a href=\"modelica://Collision.insideTriangleMesh\">ray parity</a>.</li><li>For the cited Hunt-Crossley contact law and regularized sliding friction, see <a href=\"modelica://MultiBodyLie.Collision.contactForce\">MultiBodyLie.Collision.contactForce</a>.</li></ul></html>"));
end Collision;
