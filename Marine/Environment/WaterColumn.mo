within Marine.Environment;
model WaterColumn "Tidal current, wind waves and hydrostatic pressure"
  constant Real pi=Modelica.Constants.pi,g=Modelica.Constants.g_n,rho=1025;
  input Real shelterFactor=Current.shelterAt(pos) "Mean-flow multiplier, one for unsheltered water";
  input Real seaClockOffset=0 "Seconds added to local time for incident sea phase";
  input Real seaState=-1 "Negative selects the legacy fetch-based sea";
  input Real pos[3];
  parameter Real profile[:,3]=Current.profile "Depth below mean sea level [m], east and north residual current [m/s]";
  parameter Real tidePhase=0.5, currentPhase=1.2, tidalSpeed=0.12;
  input Real windSpeed "10 m wind speed, m/s";
  input Real windDirection "Direction toward which wind blows, radians from east";
  parameter Real fetch=45000, waterDepth=SpectralSea.nearshoreDepth;
  Real wind(stateSelect=StateSelect.always,start=11,fixed=true), bearing(stateSelect=StateSelect.always,start=0.45,fixed=true);
  discrete Real tile[SpectralSea.N,SpectralSea.N],probe[5];
  discrete output Real surfaceSlope[2];
  Real surface, tide, velocity[3], gaugePressure, meanFlow[2], shelter,meanCurrent[3],tidalEast;
equation
  der(wind)=(windSpeed-wind)/20;
  der(bearing)=atan2(sin(windDirection-bearing),cos(windDirection-bearing))/18;
  tide=0.6*sin(2*pi*(time+seaClockOffset)/44714+tidePhase);
  when sample(0,0.2) then
    (tile,probe,surfaceSlope)=SpectralSea.field(time+seaClockOffset,
      if seaState<0 then wind else windSpeed,if seaState<0 then bearing else windDirection,
      fetch,waterDepth,tide,pos,seaState);
  end when;
  tidalEast=tidalSpeed*sin(2*pi*(time+seaClockOffset)/44714+currentPhase);
  for j in 1:2 loop
    meanFlow[j]=Modelica.Math.Vectors.interpolate(profile[:,1],profile[:,j+1],min(max(-pos[3],profile[1,1]),profile[size(profile,1),1]));
  end for;
  shelter=shelterFactor;
  meanCurrent=Current.meanVelocity(pos[3],tide,wind,bearing,tidalEast,meanFlow,shelter);
  surface=tide+probe[1];
  velocity=meanCurrent+probe[2:4];
  gaugePressure=rho*g*max(0,tide-pos[3]+probe[5]);
end WaterColumn;
