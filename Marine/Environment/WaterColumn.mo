within Marine.Environment;
model WaterColumn "Tidal current, wind waves and hydrostatic pressure"
  constant Real pi=Modelica.Constants.pi;
  constant Real rho=1025,g=Modelica.Constants.g_n;
  input Real pos[3];
  parameter Real profile[:,3]=[0,0.26,0.07; 8,0.19,0.04; 16,0.08,0.02] "Depth below mean sea level [m], east and north residual current [m/s]";
  parameter Real tidePhase=0.5, currentPhase=1.2, tidalSpeed=0.12;
  input Real windSpeed "10 m wind speed, m/s";
  input Real windDirection "Direction toward which wind blows, radians from east";
  parameter Real fetch=45000, waterDepth=13.7;
  Real wind(stateSelect=StateSelect.always,start=11,fixed=true), bearing(stateSelect=StateSelect.always,start=0.45,fixed=true);
  discrete Real tile[SpectralSea.N,SpectralSea.N],probe[5];
  Real surface, tide, velocity[3], gaugePressure, meanFlow[2], shelter;
equation
  der(wind)=(windSpeed-wind)/20;
  der(bearing)=atan2(sin(windDirection-bearing),cos(windDirection-bearing))/18;
  tide=0.6*sin(2*pi*time/44714+tidePhase);
  when sample(0,0.2) then
    (tile,probe)=SpectralSea.field(time,wind,bearing,fetch,waterDepth,tide,pos);
  end when;
  for j in 1:2 loop
    meanFlow[j]=Modelica.Math.Vectors.interpolate(profile[:,1],profile[:,j+1],min(max(-pos[3],profile[1,1]),profile[size(profile,1),1]));
  end for;
  surface=tide+probe[1];
  shelter=1-0.78*(0.5+0.5*tanh(pos[1]-1))*(0.5+0.5*tanh(28-pos[1]))*(0.5+0.5*tanh(4-abs(pos[2])))*(0.5+0.5*tanh(-6-pos[3]));
  velocity=shelter*({meanFlow[1]+tidalSpeed*sin(2*pi*time/44714+currentPhase),meanFlow[2],0}
    +0.014*wind*exp(min(0,pos[3]-tide)/2.2)*{cos(bearing),sin(bearing),0})
    + probe[2:4];
  gaugePressure=rho*g*max(0,tide-pos[3]+probe[5]);
end WaterColumn;
