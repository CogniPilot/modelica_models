package Geodesy
  function localNorthEastToLatLon
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
end Geodesy;
