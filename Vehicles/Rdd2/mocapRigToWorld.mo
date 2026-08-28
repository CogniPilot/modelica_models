within Vehicles.Rdd2;

function mocapRigToWorld
  "Map a motion-capture pose from the rig frame into the estimator world frame"
  input Real positionWorldEnu_m[3]
    "The pose the vehicle truly holds, in the estimator world frame. The plant
     generates truth in that frame, so this function takes the composition
     world -> rig -> world rather than the rig leg alone; when the survey is
     exact the composition is the identity, which is the property the
     degenerate handoff test asserts.";
  input Real quaternionWorldBody[4];
  input Real rigOriginWorldEnu_m[3]
    "Where the rig's own origin sits in the estimator world frame";
  input Real surveyOffsetWorldEnu_m[3]
    "Error in the surveyed rig origin: what the survey believes minus where
     the rig actually is. Zero means a perfect survey";
  input Real surveyHeadingError_rad
    "Error in the surveyed rig heading, about the world up axis";
  output Real measuredPositionWorldEnu_m[3];
  output Real measuredQuaternionWorldBody[4];
protected
  Real relative_m[3];
  Real cosHeading;
  Real sinHeading;
  Real rotated_m[3];
  Real headingQuaternion[4];
algorithm
  // ONE WORLD FRAME, TWO TRANSFORMS INTO IT. The estimator's world frame is
  // the local East-North-Up tangent frame anchored at the mission's
  // Geodesy.GeodeticOrigin datum, and it is the ONLY frame the filter's state
  // lives in. GPS reaches it through that datum, by the same
  // Geodesy.geodeticToLocalEnu path the global waypoint routes already use.
  // Motion capture reaches it through this function.
  //
  // PARAMETERIZED AS POSITION PLUS HEADING, not as a full SE(3). That is what
  // a rig survey realistically delivers: a calibration wand establishes a
  // gravity-aligned ground plane, so the levelling residual is small and
  // systematically removed, while where the rig origin sits and which way its
  // axes point relative to true east and north are surveyed quantities with
  // real error. Roll and pitch misalignment are assumed calibrated out and
  // that assumption is stated rather than buried; a rig that cannot level
  // itself needs the full SE(3) form and this function's signature would grow
  // by two angles.
  //
  // THE PLACEMENT IS FIXED AND KNOWN HERE. Estimating rig extrinsics online is
  // deliberately out of scope: it is an augmented-state problem with its own
  // observability conditions and its own failure modes. This function is where
  // that path would land -- the placement enters as arguments rather than as
  // parameters read from an outer scope, so a future estimate can be passed in
  // without changing any caller's structure.
  //
  // A survey error is not a modelling nicety. It is the reason a source
  // handoff is nontrivial: two sources that disagree about where the world's
  // origin is will disagree about where the vehicle is by exactly that much,
  // and the filter must either correct it through the innovation gate or
  // refuse it by name. It must never re-anchor the state to make the
  // disagreement go away.
  relative_m := positionWorldEnu_m - rigOriginWorldEnu_m;
  cosHeading := cos(surveyHeadingError_rad);
  sinHeading := sin(surveyHeadingError_rad);
  // A rotation about world up by the heading survey error. Composing the true
  // inverse placement with the surveyed forward placement leaves exactly this
  // rotation about the rig origin, plus the position offset.
  rotated_m := {
    cosHeading * relative_m[1] - sinHeading * relative_m[2],
    sinHeading * relative_m[1] + cosHeading * relative_m[2],
    relative_m[3]};
  measuredPositionWorldEnu_m := rigOriginWorldEnu_m + surveyOffsetWorldEnu_m
    + rotated_m;
  // The same heading error rotates the reported attitude. Left-multiplied,
  // because it is a rotation of the world frame the rig reports in and not of
  // the body.
  headingQuaternion := {
    cos(0.5 * surveyHeadingError_rad),
    0.0,
    0.0,
    sin(0.5 * surveyHeadingError_rad)};
  measuredQuaternionWorldBody := LieGroups.SO3.Quat.normalize(
    LieGroups.SO3.Quat.product(headingQuaternion, quaternionWorldBody));
end mocapRigToWorld;
