within Tests;
model RegistrationWeightedTests
  import RegistrationWeighted = Vision.Registration.RegistrationWeighted;
  import FitRigidPointPairs = Vision.Registration.FitRigidPointPairs;
  Real source[8,3]; Real target[8,3]; Real mask[8]; Real Cs[8,3,3]; Real Ct[8,3,3];
  Real seedR[3,3]; Real seedT[3]; Real R[3,3]; Real t[3]; Boolean valid;
  Real fitAccepted; Real reason; Real count; Real invalid; Real rank; Real cost; Real rms; Real gap;
  Real centroidS[3]; Real centroidT[3]; Real H[6,6]; Real gradient[6]; Real before; Real after;
  Boolean normalValid; Real truthT[3];
algorithm
when initial() then
  source := [-1,-1,2;1,-1,2;-1,1,3;1,1,3;-2,-1,8;2,-1,8;-2,1,9;2,1,9];
  mask := ones(8); truthT := {0.08,-0.03,0.12};
  for i in 1:8 loop
    target[i,:] := source[i,:]+truthT;
    Cs[i,:,:] := diagonal({0.000025,0.000025,if i <= 4 then 0.0001 else 0.25});
    Ct[i,:,:] := Cs[i,:,:];
    if i > 4 then target[i,3] := target[i,3]+(if mod(i,2)==0 then 0.6 else -0.6); end if;
  end for;
  (fitAccepted,reason,seedR,seedT,count,invalid,rank,cost,rms,gap,centroidS,centroidT) :=
    FitRigidPointPairs(source,target,mask,8,100,1e-8,1e6);
  assert(fitAccepted > 0.5,"Valid geometric seed");
  (normalValid,before,H,gradient,rms) := RegistrationWeighted.WeightedNormal(source,target,mask,Cs,Ct,seedR,seedT,1e-10);
  (valid,R,t,rms) := RegistrationWeighted.WeightedRefine(source,target,mask,Cs,Ct,seedR,seedT);
  (normalValid,after,H,gradient,rms) := RegistrationWeighted.WeightedNormal(source,target,mask,Cs,Ct,R,t,1e-10);
  assert(valid and normalValid and after < before*0.1,"Weighted objective must improve");
  assert(sqrt((t-truthT)*(t-truthT)) < 0.01,"Noisy far depth must not bias translation");
  assert(sum((R-identity(3)).^2) < 1e-4,"Noisy far depth must not bias attitude");
  Cs[1,:,:] := zeros(3,3); Ct[1,:,:] := zeros(3,3);
  (valid,R,t,rms) := RegistrationWeighted.WeightedRefine(source,target,mask,Cs,Ct,seedR,seedT);
  assert(not valid,"Singular covariance must refuse refinement");
end when;
end RegistrationWeightedTests;
