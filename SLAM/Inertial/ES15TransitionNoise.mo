within SLAM.Inertial;
// World-additive p/v, right-local theta, body ba/bg: ES15Dynamics convention.
// Caller owns dt admission. This function does not extend the supported interval.
function ES15TransitionNoise
  input Real F[15,15];
  input Real G[15,12];
  input Real dt;
  input Real density[12];
  output Real Phi[15,15];
  output Real Q[15,15];
protected
  constant Real fraction[3] = {0.5-sqrt(15.0)/10.0,0.5,0.5+sqrt(15.0)/10.0};
  constant Real weight[3] = {5.0/18.0,4.0/9.0,5.0/18.0};
  Real A[15,15]; Real A2[15,15]; Real A3[15,15];
  Real noiseTransition[3,15,15]; Real B[3,15,12];
  Real nodeNoise[3,15,15];
algorithm
  A := F*dt;
  A2 := A*A;
  A3 := A2*A;
  Phi := identity(15)+A+0.5*A2+A3/6.0;
  for node in 1:3 loop
    for i in 1:15 loop
      for j in 1:15 loop
        noiseTransition[node,i,j] := (if i == j then 1.0 else 0.0)
          +fraction[node]*A[i,j]+0.5*fraction[node]^2*A2[i,j]
          +fraction[node]^3*A3[i,j]/6.0;
      end for;
      for j in 1:12 loop
        B[node,i,j] := (noiseTransition[node,i,1]*G[1,j]+noiseTransition[node,i,2]*G[2,j]+noiseTransition[node,i,3]*G[3,j]+noiseTransition[node,i,4]*G[4,j]+noiseTransition[node,i,5]*G[5,j]+noiseTransition[node,i,6]*G[6,j]+noiseTransition[node,i,7]*G[7,j]+noiseTransition[node,i,8]*G[8,j]+noiseTransition[node,i,9]*G[9,j]+noiseTransition[node,i,10]*G[10,j]+noiseTransition[node,i,11]*G[11,j]+noiseTransition[node,i,12]*G[12,j]+noiseTransition[node,i,13]*G[13,j]+noiseTransition[node,i,14]*G[14,j]+noiseTransition[node,i,15]*G[15,j])*density[j];
      end for;
    end for;
    // Complete every B row before the Gram matrix reads B[node,j,:].
    for i in 1:15 loop
      for j in 1:15 loop
        nodeNoise[node,i,j] := (B[node,i,1]*B[node,j,1]+B[node,i,2]*B[node,j,2]+B[node,i,3]*B[node,j,3]+B[node,i,4]*B[node,j,4]+B[node,i,5]*B[node,j,5]+B[node,i,6]*B[node,j,6]+B[node,i,7]*B[node,j,7]+B[node,i,8]*B[node,j,8]+B[node,i,9]*B[node,j,9]+B[node,i,10]*B[node,j,10]+B[node,i,11]*B[node,j,11]+B[node,i,12]*B[node,j,12]);
      end for;
    end for;
  end for;
  for i in 1:15 loop
    for j in 1:15 loop
      Q[i,j] := dt*(weight[1]*nodeNoise[1,i,j]+weight[2]*nodeNoise[2,i,j]+weight[3]*nodeNoise[3,i,j]);
    end for;
  end for;
end ES15TransitionNoise;
