within Geodesy;

package WMM2025
  "NOAA World Magnetic Model 2025-2030"

  constant Integer MaximumDegree = 12;
  constant Real pi = 3.1415926535897932384626433832795;
  constant Real Wgs84A_m = 6378137.0;
  constant Real Wgs84Flattening = 1.0 / 298.257223563;
  constant Real Wgs84EccentricitySquared =
    Wgs84Flattening * (2.0 - Wgs84Flattening);
  constant Real GeomagneticReferenceRadius_m = 6371200.0;
  // NOAA WMM.COF, epoch 2025.0. Hoisted to package scope so Modelica
  // runtimes materialize the immutable table once rather than rebuilding a
  // nested array literal on every coefficient access inside the evaluator.
  // Columns are g, h, dg/dyear, dh/dyear in nT and nT/year, ordered by
  // n=1..12 and m=0..n.
    constant Real coefficients[90, 4] = [
      -29351.8,0.0,12.0,0.0;
      -1410.8,4545.4,9.7,-21.5;
      -2556.6,0.0,-11.6,0.0;
      2951.1,-3133.6,-5.2,-27.7;
      1649.3,-815.1,-8.0,-12.1;
      1361.0,0.0,-1.3,0.0;
      -2404.1,-56.6,-4.2,4.0;
      1243.8,237.5,0.4,-0.3;
      453.6,-549.5,-15.6,-4.1;
      895.0,0.0,-1.6,0.0;
      799.5,278.6,-2.4,-1.1;
      55.7,-133.9,-6.0,4.1;
      -281.1,212.0,5.6,1.6;
      12.1,-375.6,-7.0,-4.4;
      -233.2,0.0,0.6,0.0;
      368.9,45.4,1.4,-0.5;
      187.2,220.2,0.0,2.2;
      -138.7,-122.9,0.6,0.4;
      -142.0,43.0,2.2,1.7;
      20.9,106.1,0.9,1.9;
      64.4,0.0,-0.2,0.0;
      63.8,-18.4,-0.4,0.3;
      76.9,16.8,0.9,-1.6;
      -115.7,48.8,1.2,-0.4;
      -40.9,-59.8,-0.9,0.9;
      14.9,10.9,0.3,0.7;
      -60.7,72.7,0.9,0.9;
      79.5,0.0,0.0,0.0;
      -77.0,-48.9,-0.1,0.6;
      -8.8,-14.4,-0.1,0.5;
      59.3,-1.0,0.5,-0.8;
      15.8,23.4,-0.1,0.0;
      2.5,-7.4,-0.8,-1.0;
      -11.1,-25.1,-0.8,0.6;
      14.2,-2.3,0.8,-0.2;
      23.2,0.0,-0.1,0.0;
      10.8,7.1,0.2,-0.2;
      -17.5,-12.6,0.0,0.5;
      2.0,11.4,0.5,-0.4;
      -21.7,-9.7,-0.1,0.4;
      16.9,12.7,0.3,-0.5;
      15.0,0.7,0.2,-0.6;
      -16.8,-5.2,0.0,0.3;
      0.9,3.9,0.2,0.2;
      4.6,0.0,0.0,0.0;
      7.8,-24.8,-0.1,-0.3;
      3.0,12.2,0.1,0.3;
      -0.2,8.3,0.3,-0.3;
      -2.5,-3.3,-0.3,0.3;
      -13.1,-5.2,0.0,0.2;
      2.4,7.2,0.3,-0.1;
      8.6,-0.6,-0.1,-0.2;
      -8.7,0.8,0.1,0.4;
      -12.9,10.0,-0.1,0.1;
      -1.3,0.0,0.1,0.0;
      -6.4,3.3,0.0,0.0;
      0.2,0.0,0.1,0.0;
      2.0,2.4,0.1,-0.2;
      -1.0,5.3,0.0,0.1;
      -0.6,-9.1,-0.3,-0.1;
      -0.9,0.4,0.0,0.1;
      1.5,-4.2,-0.1,0.0;
      0.9,-3.8,-0.1,-0.1;
      -2.7,0.9,0.0,0.2;
      -3.9,-9.1,0.0,0.0;
      2.9,0.0,0.0,0.0;
      -1.5,0.0,0.0,0.0;
      -2.5,2.9,0.0,0.1;
      2.4,-0.6,0.0,0.0;
      -0.6,0.2,0.0,0.1;
      -0.1,0.5,-0.1,0.0;
      -0.6,-0.3,0.0,0.0;
      -0.1,-1.2,0.0,0.1;
      1.1,-1.7,-0.1,0.0;
      -1.0,-2.9,-0.1,0.0;
      -0.2,-1.8,-0.1,0.0;
      2.6,-2.3,-0.1,0.0;
      -2.0,0.0,0.0,0.0;
      -0.2,-1.3,0.0,0.0;
      0.3,0.7,0.0,0.0;
      1.2,1.0,0.0,-0.1;
      -1.3,-1.4,0.0,0.1;
      0.6,0.0,0.0,0.0;
      0.6,0.6,0.1,0.0;
      0.5,-0.1,0.0,0.0;
      -0.1,0.8,0.0,0.0;
      -0.4,0.1,0.0,0.0;
      -0.2,-1.0,-0.1,0.0;
      -1.3,0.1,0.0,0.0;
      -0.7,0.2,-0.1,-0.1];

  function magneticFieldEnu
    "Evaluate WMM2025 at WGS84 ellipsoid altitude and return ENU tesla"
    input Real latitude_deg(unit = "deg");
    input Real longitude_deg(unit = "deg");
    input Real altitudeEllipsoid_m(unit = "m");
    input Real decimalYear = 2026.0;
    output Real magneticFieldWorldEnu_T[3](each unit = "T");
  protected
    Real latitudeGeodetic_rad;
    Real longitude_rad;
    Real sinLatitudeGeodetic;
    Real cosLatitudeGeodetic;
    Real primeVerticalRadius_m;
    Real cylindricalRadius_m;
    Real z_m;
    Real radius_m;
    Real latitudeSpherical_rad;
    Real sinLatitudeSpherical;
    Real cosLatitudeSpherical;
    Real deltaYear;
    Real g[13, 13];
    Real h[13, 13];
    Real P[13, 13];
    Real dP[13, 13];
    Real sinMultipleLongitude[13];
    Real cosMultipleLongitude[13];
    Real basePower;
    Real radialPower;
    Real f1;
    Real f2;
    Real northSpherical_nT;
    Real east_nT;
    Real downSpherical_nT;
    Real north_nT;
    Real down_nT;
    Real cosDeltaLatitude;
    Real sinDeltaLatitude;
    Integer coefficientRow;
  algorithm
    latitudeGeodetic_rad := latitude_deg * pi / 180.0;
    longitude_rad := longitude_deg * pi / 180.0;
    sinLatitudeGeodetic := sin(latitudeGeodetic_rad);
    cosLatitudeGeodetic := cos(latitudeGeodetic_rad);
    primeVerticalRadius_m := Wgs84A_m / sqrt(1.0
      - Wgs84EccentricitySquared * sinLatitudeGeodetic
        * sinLatitudeGeodetic);
    cylindricalRadius_m := (primeVerticalRadius_m + altitudeEllipsoid_m)
      * cosLatitudeGeodetic;
    z_m := (primeVerticalRadius_m * (1.0 - Wgs84EccentricitySquared)
      + altitudeEllipsoid_m) * sinLatitudeGeodetic;
    radius_m := sqrt(cylindricalRadius_m * cylindricalRadius_m + z_m * z_m);
    latitudeSpherical_rad := asin(z_m / radius_m);
    sinLatitudeSpherical := sin(latitudeSpherical_rad);
    cosLatitudeSpherical := cos(latitudeSpherical_rad);
    deltaYear := decimalYear - 2025.0;

    g := zeros(13, 13);
    h := zeros(13, 13);
    coefficientRow := 0;
    for n in 1:MaximumDegree loop
      for m in 0:n loop
        coefficientRow := coefficientRow + 1;
        g[n + 1, m + 1] := coefficients[coefficientRow, 1]
          + deltaYear * coefficients[coefficientRow, 3];
        h[n + 1, m + 1] := coefficients[coefficientRow, 2]
          + deltaYear * coefficients[coefficientRow, 4];
      end for;
    end for;

    // Schmidt semi-normalized associated Legendre functions and latitude
    // derivatives. This fixed degree-12 recurrence is the Modelica port of
    // the NOAA evaluation used by the reference C implementation.
    P := zeros(13, 13);
    dP := zeros(13, 13);
    P[1, 1] := 1.0;
    P[2, 1] := sinLatitudeSpherical;
    dP[2, 1] := cosLatitudeSpherical;
    for n in 2:MaximumDegree loop
      f1 := (2.0 * n - 1.0) / n;
      f2 := (n - 1.0) / n;
      P[n + 1, 1] := f1 * sinLatitudeSpherical * P[n, 1]
        - f2 * P[n - 1, 1];
      dP[n + 1, 1] := n * (P[n, 1]
        - sinLatitudeSpherical * P[n + 1, 1])
          / cosLatitudeSpherical;
    end for;
    for m in 1:MaximumDegree loop
      // Schmidt quasi-normalization has a distinct m=1 seed. Applying the
      // general diagonal recurrence at m=1 incorrectly scales every
      // non-zonal term by 1/sqrt(2).
      P[m + 1, m + 1] := if m == 1 then cosLatitudeSpherical
        else P[m, m] * cosLatitudeSpherical
          * sqrt((2.0 * m - 1.0) / (2.0 * m));
      dP[m + 1, m + 1] := -m * sinLatitudeSpherical
        * P[m + 1, m + 1] / cosLatitudeSpherical;
      if m < MaximumDegree then
        P[m + 2, m + 1] := sinLatitudeSpherical * sqrt(2.0 * m + 1.0)
          * P[m + 1, m + 1];
        dP[m + 2, m + 1] := (sqrt(2.0 * m + 1.0)
          * P[m + 1, m + 1] - sinLatitudeSpherical * (m + 1.0)
            * P[m + 2, m + 1]) / cosLatitudeSpherical;
      end if;
      for n in m + 2:MaximumDegree loop
        f1 := (2.0 * n - 1.0) / sqrt((n + m) * (n - m));
        f2 := sqrt((n - m - 1.0) * (n + m - 1.0)
          / ((n + m) * (n - m)));
        P[n + 1, m + 1] := sinLatitudeSpherical * f1
          * P[n, m + 1] - f2 * P[n - 1, m + 1];
        dP[n + 1, m + 1] := (sqrt((n + m) * (n - m))
          * P[n, m + 1] - n * sinLatitudeSpherical
            * P[n + 1, m + 1]) / cosLatitudeSpherical;
      end for;
    end for;

    sinMultipleLongitude := zeros(13);
    cosMultipleLongitude := zeros(13);
    cosMultipleLongitude[1] := 1.0;
    for m in 1:MaximumDegree loop
      sinMultipleLongitude[m + 1] := sinMultipleLongitude[m]
          * cos(longitude_rad) + cosMultipleLongitude[m]
          * sin(longitude_rad);
      cosMultipleLongitude[m + 1] := cosMultipleLongitude[m]
          * cos(longitude_rad) - sinMultipleLongitude[m]
          * sin(longitude_rad);
    end for;

    northSpherical_nT := 0.0;
    east_nT := 0.0;
    downSpherical_nT := 0.0;
    basePower := GeomagneticReferenceRadius_m / radius_m;
    radialPower := basePower * basePower;
    for n in 1:MaximumDegree loop
      radialPower := radialPower * basePower;
      for m in 0:n loop
        northSpherical_nT := northSpherical_nT - radialPower
          * (g[n + 1, m + 1] * cosMultipleLongitude[m + 1]
            + h[n + 1, m + 1] * sinMultipleLongitude[m + 1])
          * dP[n + 1, m + 1];
        east_nT := east_nT + radialPower / cosLatitudeSpherical * m
          * (g[n + 1, m + 1] * sinMultipleLongitude[m + 1]
            - h[n + 1, m + 1] * cosMultipleLongitude[m + 1])
          * P[n + 1, m + 1];
        downSpherical_nT := downSpherical_nT
          - (n + 1.0) * radialPower
          * (g[n + 1, m + 1] * cosMultipleLongitude[m + 1]
            + h[n + 1, m + 1] * sinMultipleLongitude[m + 1])
          * P[n + 1, m + 1];
      end for;
    end for;
    cosDeltaLatitude := cos(latitudeSpherical_rad - latitudeGeodetic_rad);
    sinDeltaLatitude := sin(latitudeSpherical_rad - latitudeGeodetic_rad);
    north_nT := northSpherical_nT * cosDeltaLatitude
      - downSpherical_nT * sinDeltaLatitude;
    down_nT := northSpherical_nT * sinDeltaLatitude
      + downSpherical_nT * cosDeltaLatitude;
    magneticFieldWorldEnu_T := {east_nT, north_nT, -down_nT} * 1.0e-9;
  end magneticFieldEnu;

  function validInputs
    "Report whether inputs lie inside the WMM2025 supported domain"
    input Real latitude_deg(unit = "deg");
    input Real altitudeEllipsoid_m(unit = "m");
    input Real decimalYear;
    output Boolean valid;
  algorithm
    valid := abs(latitude_deg) < 89.999999
      and altitudeEllipsoid_m >= -1000.0
      and altitudeEllipsoid_m <= 850000.0
      and decimalYear >= 2025.0 and decimalYear <= 2030.0;
  end validInputs;

  annotation(Documentation(info = "<html>
    <p>Pure-Modelica port of the public-domain WMM2025 degree/order-12
    spherical-harmonic evaluation. The coefficient epoch is 2025.0 and the
    model is valid through 2030.0. Input altitude is above the WGS84 ellipsoid.
    The model publishes components as north/east/down nanotesla; this package
    returns East-North-Up tesla for direct use by the navigation models.</p>
    <p>The World Magnetic Model is produced jointly by the NOAA National
    Centers for Environmental Information (NCEI) and the British Geological
    Survey (BGS); both are credited. The model and its software are in the
    public domain in the United States. Neither producer endorses this port.
    Coefficients and validity window are from the WMM2025 Technical Report,
    <a href='https://www.ncei.noaa.gov/products/world-magnetic-model'>
    https://www.ncei.noaa.gov/products/world-magnetic-model</a>.</p>
  </html>"));
end WMM2025;
