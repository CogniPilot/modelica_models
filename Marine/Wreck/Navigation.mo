within Marine.Wreck;
model Navigation "Perception and navigation FMU sampled by the co-simulation master"
  input Real stamp, truth[18], priorImage[864],prior[21],keyHistory[48,180];
  input Real idleCamera(start=0),useSonar(start=1);
  final parameter Real beam[1017,3]=Perception.directions();
  output Real sensor[91],elev[100],image[864],estimate[21],keyframes[48,180],features[24,5],ping[65,32];
equation
  (sensor,elev,image,estimate,keyframes,features,ping)=Perception.sampleFrame(stamp,truth,beam,zeros(1017),0,priorImage,prior,keyHistory,idleCamera<.5,idleCamera<.5,useSonar>.5);
end Navigation;
