within Vision.Registration;
model RigidPointRegistration
  import FitRigidPointPairs = Vision.Registration.FitRigidPointPairs;

  parameter Integer capacity = 14400;
  parameter Real coordinateLimit = 1e6;
  parameter Real rankTolerance = 1e-8;
  parameter Real maximumRms = 0.02;
  input Real activeCount = 0.0;
  input Real sourcePoint[capacity,3] = zeros(capacity,3);
  input Real targetPoint[capacity,3] = zeros(capacity,3);
  // Exactly 0 disables a pair; exactly 1 requests a finite matched pair.
  input Real pairEnabled[capacity] = ones(capacity);
  output Real accepted;
  // 0 accepted; 1 domain; 2 invalid pair; 3 insufficient; 4 degeneracy;
  // 5 eigensolver convergence; 6 fitted RMS gate. Rejection is identity/zero.
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
equation
  (accepted,rejectionReason,rotation,translation,validCount,invalidCount,rank,cost,rms,eigenGap,sourceCentroid,targetCentroid) =
    FitRigidPointPairs(sourcePoint,targetPoint,pairEnabled,activeCount,coordinateLimit,rankTolerance,maximumRms);
end RigidPointRegistration;
