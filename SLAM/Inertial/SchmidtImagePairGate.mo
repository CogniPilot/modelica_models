within SLAM.Inertial;
model SchmidtImagePairGate
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  input Real referenceAvailable = 0.0;
  input Real referenceUsed = 0.0;
  input Real referenceEpoch = 0.0;
  input Real currentEpoch = 0.0;
  input Real lastUsedEpoch = -1.0;
  output Real valid;
  output Real eligible;
  output Real captureFresh;
equation
  valid = if noEvent((SLAMExactRealEqual(referenceAvailable,0.0) or SLAMExactRealEqual(referenceAvailable,1.0))
    and (SLAMExactRealEqual(referenceUsed,0.0) or SLAMExactRealEqual(referenceUsed,1.0))
    and currentEpoch >= 0.0 and currentEpoch <= 9007199254740991.0
    and SLAMExactRealEqual(currentEpoch,floor(currentEpoch))
    and lastUsedEpoch >= -1.0 and lastUsedEpoch <= 9007199254740991.0
    and SLAMExactRealEqual(lastUsedEpoch,floor(lastUsedEpoch))
    and (SLAMExactRealEqual(referenceAvailable,0.0) and SLAMExactRealEqual(referenceUsed,0.0)
      or SLAMExactRealEqual(referenceAvailable,1.0) and referenceEpoch >= 0.0
        and referenceEpoch <= 9007199254740991.0 and SLAMExactRealEqual(referenceEpoch,floor(referenceEpoch))
        and (SLAMExactRealEqual(referenceUsed,0.0) and referenceEpoch > lastUsedEpoch
          or SLAMExactRealEqual(referenceUsed,1.0) and referenceEpoch <= lastUsedEpoch))) then 1.0 else 0.0;
  eligible = if noEvent(valid > 0.5 and SLAMExactRealEqual(referenceAvailable,1.0)
    and SLAMExactRealEqual(referenceUsed,0.0) and currentEpoch > referenceEpoch) then 1.0 else 0.0;
  captureFresh = if noEvent(valid > 0.5 and currentEpoch > lastUsedEpoch
    and (SLAMExactRealEqual(referenceAvailable,0.0) or currentEpoch > referenceEpoch)) then 1.0 else 0.0;
end SchmidtImagePairGate;
