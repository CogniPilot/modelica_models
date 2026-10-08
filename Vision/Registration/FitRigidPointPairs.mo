within Vision.Registration;
// Three ordered full-domain passes retain compact runtime loop ownership.
// Disabled or invalid pairs never enter multiplication, even with NaN inputs.
function FitRigidPointPairs
  import RegistrationEigen4 = Vision.Registration.RegistrationEigen4;

  input Real sourcePoint[:,:];
  input Real targetPoint[:,:];
  input Real pairEnabled[:];
  input Real activeCount;
  input Real coordinateLimit;
  input Real rankTolerance;
  input Real maximumRms;
  output Real accepted;
  output Real rejectionReason;
  output Real rotation[3,3];
  output Real translation[3];
  output Real validCount;
  output Real invalidCount;
  output Real rank;
  output Real cost;
  output Real rms;
  output Real eigenGap;
  output Real sourceCentroid[3];
  output Real targetCentroid[3];
protected
  Boolean mask[size(pairEnabled,1)];
  Boolean domainValid;
  Real crossCovariance[3,3]; Real sourceCovariance[4,4];
  Real horn[4,4]; Real hornValues[4]; Real sourceValues[4];
  Real quaternion[4]; Real unusedVector[4];
  Real unusedGap; Real hornResidual; Real sourceResidual;
  Real sourceScale; Real hornScale;
  Real candidateRotation[3,3]; Real candidateTranslation[3];
  Real error;
algorithm
  domainValid := activeCount >= 0.0 and activeCount <= size(pairEnabled,1) and floor(activeCount) >= activeCount and
    size(pairEnabled,1) >= 3 and size(sourcePoint,1) == size(pairEnabled,1) and size(targetPoint,1) == size(pairEnabled,1) and
    size(sourcePoint,2) == 3 and size(targetPoint,2) == 3 and
    coordinateLimit > 0.0 and coordinateLimit <= 1e6 and rankTolerance >= 1e-12 and rankTolerance <= 1e-2 and
    maximumRms >= 0.0 and maximumRms <= 1e6;
  mask := fill(false,size(pairEnabled,1));
  validCount := 0.0; invalidCount := 0.0;
  sourceCentroid := zeros(3); targetCentroid := zeros(3);
  for i in 1:size(pairEnabled,1) loop
    if domainValid and i <= activeCount then
      if pairEnabled[i] >= 1.0 and pairEnabled[i] <= 1.0 and
        abs(sourcePoint[i,1]) <= coordinateLimit and abs(sourcePoint[i,2]) <= coordinateLimit and abs(sourcePoint[i,3]) <= coordinateLimit and
        abs(targetPoint[i,1]) <= coordinateLimit and abs(targetPoint[i,2]) <= coordinateLimit and abs(targetPoint[i,3]) <= coordinateLimit then
        mask[i] := true;
        validCount := validCount+1.0;
        for a in 1:3 loop
          sourceCentroid[a] := sourceCentroid[a]+sourcePoint[i,a];
          targetCentroid[a] := targetCentroid[a]+targetPoint[i,a];
        end for;
      elseif not (pairEnabled[i] >= 0.0 and pairEnabled[i] <= 0.0) then
        invalidCount := invalidCount+1.0;
      end if;
    end if;
  end for;
  sourceCentroid := sourceCentroid/max(validCount,1.0);
  targetCentroid := targetCentroid/max(validCount,1.0);
  crossCovariance := zeros(3,3); sourceCovariance := zeros(4,4);
  for i in 1:size(pairEnabled,1) loop
    if mask[i] then
      for a in 1:3 loop
        for b in 1:3 loop
          crossCovariance[a,b] := crossCovariance[a,b]+(sourcePoint[i,a]-sourceCentroid[a])*(targetPoint[i,b]-targetCentroid[b]);
          sourceCovariance[a,b] := sourceCovariance[a,b]+(sourcePoint[i,a]-sourceCentroid[a])*(sourcePoint[i,b]-sourceCentroid[b]);
        end for;
      end for;
    end if;
  end for;
  crossCovariance := crossCovariance/max(validCount,1.0);
  sourceCovariance := sourceCovariance/max(validCount,1.0);
  horn := {{crossCovariance[1,1]+crossCovariance[2,2]+crossCovariance[3,3],crossCovariance[2,3]-crossCovariance[3,2],crossCovariance[3,1]-crossCovariance[1,3],crossCovariance[1,2]-crossCovariance[2,1]},
    {crossCovariance[2,3]-crossCovariance[3,2],crossCovariance[1,1]-crossCovariance[2,2]-crossCovariance[3,3],crossCovariance[1,2]+crossCovariance[2,1],crossCovariance[1,3]+crossCovariance[3,1]},
    {crossCovariance[3,1]-crossCovariance[1,3],crossCovariance[1,2]+crossCovariance[2,1],-crossCovariance[1,1]+crossCovariance[2,2]-crossCovariance[3,3],crossCovariance[2,3]+crossCovariance[3,2]},
    {crossCovariance[1,2]-crossCovariance[2,1],crossCovariance[1,3]+crossCovariance[3,1],crossCovariance[2,3]+crossCovariance[3,2],-crossCovariance[1,1]-crossCovariance[2,2]+crossCovariance[3,3]}};
  (hornValues,quaternion,eigenGap,hornResidual) := RegistrationEigen4(horn);
  (sourceValues,unusedVector,unusedGap,sourceResidual) := RegistrationEigen4(sourceCovariance);
  sourceScale := 0.0; hornScale := 0.0;
  for i in 1:4 loop
    sourceScale := max(sourceScale,abs(sourceValues[i]));
    hornScale := max(hornScale,abs(hornValues[i]));
  end for;
  rank := 0.0;
  for i in 1:4 loop
    rank := rank+(if sourceValues[i] > rankTolerance*sourceScale then 1.0 else 0.0);
  end for;
  candidateRotation := {{1.0-2.0*(quaternion[3]^2+quaternion[4]^2),2.0*(quaternion[2]*quaternion[3]-quaternion[1]*quaternion[4]),2.0*(quaternion[2]*quaternion[4]+quaternion[1]*quaternion[3])},
    {2.0*(quaternion[2]*quaternion[3]+quaternion[1]*quaternion[4]),1.0-2.0*(quaternion[2]^2+quaternion[4]^2),2.0*(quaternion[3]*quaternion[4]-quaternion[1]*quaternion[2])},
    {2.0*(quaternion[2]*quaternion[4]-quaternion[1]*quaternion[3]),2.0*(quaternion[3]*quaternion[4]+quaternion[1]*quaternion[2]),1.0-2.0*(quaternion[2]^2+quaternion[3]^2)}};
  candidateTranslation := zeros(3);
  for a in 1:3 loop
    candidateTranslation[a] := targetCentroid[a];
    for b in 1:3 loop
      candidateTranslation[a] := candidateTranslation[a]-candidateRotation[a,b]*sourceCentroid[b];
    end for;
  end for;
  cost := 0.0;
  for i in 1:size(pairEnabled,1) loop
    if mask[i] then
      for a in 1:3 loop
        error := candidateTranslation[a]-targetPoint[i,a];
        for b in 1:3 loop
          error := error+candidateRotation[a,b]*sourcePoint[i,b];
        end for;
        cost := cost+error*error;
      end for;
    end if;
  end for;
  rms := sqrt(max(cost,0.0)/max(validCount,1.0));
  rejectionReason := if not domainValid then 1.0 else if invalidCount > 0.0 then 2.0 else if validCount < 3.0 then 3.0
    else if rank < 2.0 or eigenGap <= rankTolerance*max(sourceScale,hornScale) then 4.0
    else if hornResidual > 1e-10 or sourceResidual > 1e-10 then 5.0
    else if not (rms >= 0.0 and rms <= maximumRms) then 6.0 else 0.0;
  accepted := if rejectionReason <= 0.0 then 1.0 else 0.0;
  rotation := if accepted > 0.0 then candidateRotation else identity(3);
  translation := if accepted > 0.0 then candidateTranslation else zeros(3);
end FitRigidPointPairs;
