within;
package Geodesy "Local-frame and geodetic conversion helpers"

  record GeodeticOrigin
    "Reference point anchoring a local East-North-Up tangent frame"
    Real latitude_deg = 0.0 "Reference latitude [deg]";
    Real longitude_deg = 0.0 "Reference longitude [deg]";
    Real altitude_m = 0.0 "Reference altitude above the datum [m]";
  end GeodeticOrigin;

  function localNorthEastToLatLon
    "Great-circle projection of a local north/east offset to geodetic lat/lon"
    input Real lat0_deg "Reference latitude [deg]";
    input Real lon0_deg "Reference longitude [deg]";
    input Real north_m "Local north displacement [m]";
    input Real east_m "Local east displacement [m]";
    input Real earth_radius_m = 6378137.0 "Spherical Earth radius [m]";
    output Real lat_lon_deg[2] "Latitude and longitude [deg]";
  protected
    constant Real pi = 3.141592653589793;
    Real lat0;
    Real lon0;
    Real distance_m;
    Real angular_distance;
    Real bearing;
    Real lat;
    Real lon;

  algorithm
    lat0 := lat0_deg * pi / 180.0;
    lon0 := lon0_deg * pi / 180.0;
    distance_m := sqrt(north_m * north_m + east_m * east_m);

    if distance_m <= 1.0e-9 then
      lat := lat0;
      lon := lon0;
    else
      angular_distance := distance_m / earth_radius_m;
      bearing := atan2(east_m, north_m);
      lat := asin(
        sin(lat0) * cos(angular_distance) +
        cos(lat0) * sin(angular_distance) * cos(bearing));
      lon := lon0 + atan2(
        sin(bearing) * sin(angular_distance) * cos(lat0),
        cos(angular_distance) - sin(lat0) * sin(lat));
    end if;

    lat_lon_deg[1] := lat * 180.0 / pi;
    lat_lon_deg[2] := lon * 180.0 / pi;
  end localNorthEastToLatLon;

  function latLonToLocalNorthEast
    "Great-circle inverse of localNorthEastToLatLon"
    input Real lat0_deg "Reference latitude [deg]";
    input Real lon0_deg "Reference longitude [deg]";
    input Real lat_deg "Target latitude [deg]";
    input Real lon_deg "Target longitude [deg]";
    input Real earth_radius_m = 6378137.0 "Spherical Earth radius [m]";
    output Real north_east_m[2] "Local north and east displacement [m]";
  protected
    constant Real pi = 3.141592653589793;
    Real lat0;
    Real lon0;
    Real lat;
    Real lon;
    Real dLon;
    Real haversine;
    Real centralAngle;
    Real distance_m;
    Real bearingY;
    Real bearingX;
    Real bearing;

  algorithm
    lat0 := lat0_deg * pi / 180.0;
    lon0 := lon0_deg * pi / 180.0;
    lat := lat_deg * pi / 180.0;
    lon := lon_deg * pi / 180.0;
    dLon := lon - lon0;

    // Central angle via the haversine, clamped to a valid arcsine domain.
    haversine := sin((lat - lat0) / 2.0) * sin((lat - lat0) / 2.0)
      + cos(lat0) * cos(lat) * sin(dLon / 2.0) * sin(dLon / 2.0);
    haversine := max(0.0, min(1.0, haversine));
    centralAngle := 2.0 * asin(sqrt(haversine));

    if centralAngle <= 1.0e-12 then
      north_east_m := {0.0, 0.0};
    else
      // Initial bearing from the origin to the target, then decompose the
      // arc length into north and east so that the forward projection above
      // recovers the target exactly.
      bearingY := sin(dLon) * cos(lat);
      bearingX := cos(lat0) * sin(lat) - sin(lat0) * cos(lat) * cos(dLon);
      bearing := atan2(bearingY, bearingX);
      distance_m := earth_radius_m * centralAngle;
      north_east_m := {distance_m * cos(bearing), distance_m * sin(bearing)};
    end if;
  end latLonToLocalNorthEast;

  function geodeticToLocalEnu
    "Project a geodetic point into the origin's East-North-Up frame"
    input GeodeticOrigin origin "Local frame reference point";
    input Real lat_deg "Target latitude [deg]";
    input Real lon_deg "Target longitude [deg]";
    input Real alt_m "Target altitude above the datum [m]";
    input Real earth_radius_m = 6378137.0 "Spherical Earth radius [m]";
    output Real enu_m[3] "East, north, and up displacement [m]";
  protected
    Real north_east_m[2];
  algorithm
    north_east_m := latLonToLocalNorthEast(
      origin.latitude_deg, origin.longitude_deg, lat_deg, lon_deg,
      earth_radius_m);
    enu_m := {north_east_m[2], north_east_m[1], alt_m - origin.altitude_m};
  end geodeticToLocalEnu;

  function localEnuToGeodetic
    "Inverse of geodeticToLocalEnu"
    input GeodeticOrigin origin "Local frame reference point";
    input Real east_m "Local east displacement [m]";
    input Real north_m "Local north displacement [m]";
    input Real up_m "Local up displacement [m]";
    input Real earth_radius_m = 6378137.0 "Spherical Earth radius [m]";
    output Real geodetic[3] "Latitude [deg], longitude [deg], altitude [m]";
  protected
    Real lat_lon_deg[2];
  algorithm
    lat_lon_deg := localNorthEastToLatLon(
      origin.latitude_deg, origin.longitude_deg, north_m, east_m,
      earth_radius_m);
    geodetic := {lat_lon_deg[1], lat_lon_deg[2], origin.altitude_m + up_m};
  end localEnuToGeodetic;

  function projectRouteToLocalEnu
    "Project a lat/lon/alt route into the origin's East-North-Up frame"
    input GeodeticOrigin origin "Local frame reference point";
    input Real globalWaypoints[:, 3]
      "Route rows are [latitude_deg, longitude_deg, altitude_m]";
    input Real earth_radius_m = 6378137.0 "Spherical Earth radius [m]";
    output Real localWaypoints[size(globalWaypoints, 1), 3]
      "Route rows are [east_m, north_m, up_m]";
  algorithm
    // Element-wise assignment over both axes so the loop totally defines the
    // output array (the eFMI lowering proves definedness per scalar element).
    for row in 1:size(globalWaypoints, 1) loop
      for col in 1:3 loop
        localWaypoints[row, col] := geodeticToLocalEnu(
          origin,
          globalWaypoints[row, 1],
          globalWaypoints[row, 2],
          globalWaypoints[row, 3],
          earth_radius_m)[col];
      end for;
    end for;
  end projectRouteToLocalEnu;

  function routeLocalEnuToGeodetic
    "Lift a local East-North-Up route to lat/lon/alt through the origin"
    input GeodeticOrigin origin "Local frame reference point";
    input Real localWaypoints[:, 3]
      "Route rows are [east_m, north_m, up_m]";
    input Real earth_radius_m = 6378137.0 "Spherical Earth radius [m]";
    output Real globalWaypoints[size(localWaypoints, 1), 3]
      "Route rows are [latitude_deg, longitude_deg, altitude_m]";
  algorithm
    // Element-wise assignment over both axes so the loop totally defines the
    // output array (the eFMI lowering proves definedness per scalar element).
    for row in 1:size(localWaypoints, 1) loop
      for col in 1:3 loop
        globalWaypoints[row, col] := localEnuToGeodetic(
          origin,
          localWaypoints[row, 1],
          localWaypoints[row, 2],
          localWaypoints[row, 3],
          earth_radius_m)[col];
      end for;
    end for;
  end routeLocalEnuToGeodetic;

  annotation(Documentation(info="<html>
    <h4>Overview</h4>
    <p>Spherical great-circle conversions between a geodetic frame and a local
    tangent frame anchored at a <code>GeodeticOrigin</code>. The local frame is
    East-North-Up (x East, y North, z Up), matching the world z-up convention of
    the rigid-body plants and the multirotor controllers.</p>
    <p><code>latLonToLocalNorthEast</code> is the exact inverse of
    <code>localNorthEastToLatLon</code>; the East-North-Up wrappers add the
    altitude channel and the route helpers project whole waypoint lists at
    parameter time. Missions with global waypoints project them once through a
    fixed mission origin; the spherical model is sub-meter accurate to tens of
    kilometres, well inside the range where re-anchoring the origin would be
    needed.</p>
  </html>"));
end Geodesy;
