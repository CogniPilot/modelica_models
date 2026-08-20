within Tests;
model GeodesyTests "Local-frame and geodetic conversion tests"
  function run
    output Boolean passed;
  protected
    constant Real pi = 3.141592653589793;
    constant Real earthRadius_m = 6378137.0;
    constant Real roundTripTolerance = 1.0e-6;
    Geodesy.GeodeticOrigin origin = Geodesy.GeodeticOrigin(
      latitude_deg = 40.4237,
      longitude_deg = -86.9212,
      altitude_m = 180.0);
    Real enu[3];
    Real geodetic[3];
    Real enuBack[3];
    Real northEast[2];
    Real latLon[2];
    Real northEastBack[2];
    Real northByOneDegree[3];
    Real expectedNorth_m;
    Real localRoute[3, 3];
    Real globalRoute[3, 3];
    Real localBack[3, 3];
    Real magneticFieldEnu_T[3];
    Boolean magneticFieldValid;
  algorithm
    // East-North-Up round trip through the geodetic frame.
    enu := {300.0, -150.0, 25.0};
    geodetic := Geodesy.localEnuToGeodetic(origin, enu[1], enu[2], enu[3]);
    enuBack := Geodesy.geodeticToLocalEnu(
      origin, geodetic[1], geodetic[2], geodetic[3]);
    assert(Tests.Assertions.maxAbsVector(enuBack - enu) < roundTripTolerance,
      "geodeticToLocalEnu did not invert localEnuToGeodetic");

    // North/east inverse recovers the forward great-circle projection.
    northEast := {1000.0, -500.0};
    latLon := Geodesy.localNorthEastToLatLon(
      origin.latitude_deg, origin.longitude_deg, northEast[1], northEast[2]);
    northEastBack := Geodesy.latLonToLocalNorthEast(
      origin.latitude_deg, origin.longitude_deg, latLon[1], latLon[2]);
    assert(
      Tests.Assertions.maxAbsVector(northEastBack - northEast)
        < roundTripTolerance,
      "latLonToLocalNorthEast did not invert localNorthEastToLatLon");

    // One degree of latitude is one Earth radius times one degree in radians,
    // straight north with no east component.
    northByOneDegree := Geodesy.geodeticToLocalEnu(
      origin, origin.latitude_deg + 1.0, origin.longitude_deg,
      origin.altitude_m);
    expectedNorth_m := earthRadius_m * pi / 180.0;
    assert(abs(northByOneDegree[2] - expectedNorth_m) < 1.0,
      "One degree of latitude did not map to the meridian arc length");
    assert(abs(northByOneDegree[1]) < 1.0e-3,
      "A pure latitude change produced an east component");
    assert(abs(northByOneDegree[3]) < 1.0e-9,
      "A pure latitude change produced an up component");

    // Whole-route projection round trip.
    localRoute := [
      0.0, 0.0, 0.0;
      100.0, 50.0, 10.0;
      -30.0, 80.0, 5.0];
    globalRoute := Geodesy.routeLocalEnuToGeodetic(origin, localRoute);
    localBack := Geodesy.projectRouteToLocalEnu(origin, globalRoute);
    assert(
      Tests.Assertions.maxAbsMatrix(localBack - localRoute)
        < roundTripTolerance,
      "projectRouteToLocalEnu did not invert routeLocalEnuToGeodetic");

    // Official NOAA WMM2025 test vector: X north, Y east, Z down at
    // 43 deg N, 93 deg E, 65 km WGS84 ellipsoid altitude, epoch 2025.0.
    magneticFieldEnu_T :=
      Geodesy.WMM2025.magneticFieldEnu(43.0, 93.0, 65000.0, 2025.0);
    magneticFieldValid :=
      Geodesy.WMM2025.validInputs(43.0, 65000.0, 2025.0);
    assert(magneticFieldValid and Tests.Assertions.maxAbsVector(
        magneticFieldEnu_T * 1.0e9
          - {210.517066, 24299.852822, -50037.923998}) < 0.01,
      "Pure-Modelica WMM2025 field did not match the NOAA reference vector");

    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Geodesy assertions did not complete");
  annotation(experiment(StartTime = 0.0, StopTime = 1.0, Tolerance = 1.0e-8,
    Interval = 0.001));
end GeodesyTests;
