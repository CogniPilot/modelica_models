within Control.Mpc.Test;

// SPDX-License-Identifier: Apache-2.0

function doubleIntegratorLinearization
  "Finite-difference horizon linearization of the ZOH double integrator"
  input Real linearizationStates[:, 2];
  input Real linearizationControls[size(linearizationStates, 1), 1];
  input Real samplePeriod(unit="s");
  input Real jacobianStep;
  output Real stateJacobians[size(linearizationStates, 1) * 2, 2];
  output Real inputJacobians[size(linearizationStates, 1) * 2, 1];
  output Real nominalNodes[size(linearizationStates, 1), 2];
protected
  Real nominal[2];
  Real column[2];
algorithm
  stateJacobians := zeros(size(linearizationStates, 1) * 2, 2);
  inputJacobians := zeros(size(linearizationStates, 1) * 2, 1);
  nominalNodes := zeros(size(linearizationStates, 1), 2);
  nominal := zeros(2);
  column := zeros(2);
  for k in 1:size(linearizationStates, 1) loop
    nominal := doubleIntegratorStep(linearizationStates[k, :],
      linearizationControls[k, :], samplePeriod);
    nominalNodes[k, :] := nominal;
    column := (doubleIntegratorStep(
      {linearizationStates[k, 1] + jacobianStep, linearizationStates[k, 2]},
      linearizationControls[k, :], samplePeriod) - nominal) / jacobianStep;
    stateJacobians[(k - 1) * 2 + 1, 1] := column[1];
    stateJacobians[(k - 1) * 2 + 2, 1] := column[2];
    column := (doubleIntegratorStep(
      {linearizationStates[k, 1], linearizationStates[k, 2] + jacobianStep},
      linearizationControls[k, :], samplePeriod) - nominal) / jacobianStep;
    stateJacobians[(k - 1) * 2 + 1, 2] := column[1];
    stateJacobians[(k - 1) * 2 + 2, 2] := column[2];
    column := (doubleIntegratorStep(linearizationStates[k, :],
      {linearizationControls[k, 1] + jacobianStep},
      samplePeriod) - nominal) / jacobianStep;
    inputJacobians[(k - 1) * 2 + 1, 1] := column[1];
    inputJacobians[(k - 1) * 2 + 2, 1] := column[2];
  end for;
  annotation(Documentation(info="<html>
    <p>Reference implementation of the consumer side of the transcription
    linearization ports: evaluate the <code>PartialDynamics</code>
    implementation at every published query point and at forward
    finite-difference perturbations of it. The perturbed points are written
    as explicit array constructors because chaining a second helper call
    inside this loop currently exceeds the compiler's function
    scalar-projection recursion limit. Once the compiler honors function redeclaration this whole
    function collapses into a <code>redeclare</code> of the prediction
    model inside the transcription.</p>
  </html>"));
end doubleIntegratorLinearization;
