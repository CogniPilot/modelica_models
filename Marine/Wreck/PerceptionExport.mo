within Marine.Wreck;
model PerceptionExport
  input Real u;
  function link
    input Real t; output Real y;
  protected
    Real s[91],e[100],im[864],state[21],graph[48,180],ft[24,5],env[10];
  algorithm
    (s,e,im,state,graph,ft):=Perception.sampleFrame(t,fill(t,18),Perception.directions(24+t),zeros(1017),0,zeros(864),zeros(21),zeros(48,180));
    env:=Perception.boatEnvironment(t,zeros(32,32),0,11,.45,0,0,0);
    y:=s[1]+e[1]+im[1]+state[1]+graph[1,1]+ft[1,1]+env[1];
  end link;
  output Real keep=link(u);
end PerceptionExport;
