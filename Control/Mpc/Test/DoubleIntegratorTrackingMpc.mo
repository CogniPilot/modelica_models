within Control.Mpc.Test;

// SPDX-License-Identifier: Apache-2.0

model DoubleIntegratorTrackingMpc
  "Receding-horizon tracking of a position step on a sampled double integrator"
  parameter Real samplePeriod(unit="s") = 0.05;
  parameter Real positionReference_m = 1.0
    "Position step target in meters";
  parameter Real jacobianStep = 1.0e-6;
  replaceable block Transcription = Control.Mpc.MultipleShooting
    constrainedby Control.Mpc.PartialTranscription
    "Selected OCP transcription";
  Transcription mpc(
    ocp=Control.Mpc.OcpSpec(
      stateCount=2,
      inputCount=1,
      horizonLength=10,
      samplePeriod=samplePeriod,
      regularization=1.0e-9),
    stateWeight={100.0, 10.0},
    inputWeight={1.0},
    terminalWeight={500.0, 50.0},
    redeclare function NlpSolver = Control.Mpc.gaussNewtonSqpStep);
  discrete Real plantState[2](each start=0.0)
    "Plant position and velocity, advanced by the same ZOH step";
  Real position_m "Plant position in meters";
  Real velocity_m_s "Plant velocity in meters per second";
  Real command_m_s2 "Applied acceleration command in meters per second squared";
equation
  mpc.currentState = {pre(plantState[1]), pre(plantState[2])};
  mpc.reference = {positionReference_m, 0.0};
  (mpc.stateJacobians, mpc.inputJacobians, mpc.nominalNodes) =
    Control.Mpc.Test.doubleIntegratorLinearization(
      mpc.linearizationStates,
      mpc.linearizationControls,
      samplePeriod,
      jacobianStep);
  position_m = plantState[1];
  velocity_m_s = plantState[2];
  command_m_s2 = mpc.command[1];
algorithm
  when sample(0.0, samplePeriod) then
    plantState := Control.Mpc.Test.doubleIntegratorStep(
      pre(plantState), mpc.command, samplePeriod);
  end when;
  annotation(experiment(StartTime=0.0, StopTime=5.0,
    Tolerance=1.0e-8, Interval=0.05),
    Documentation(info="<html>
    <p>Fully discrete clocked composition: the MPC measures the plant state
    of the previous tick, takes one warm-started Gauss-Newton step on a
    10-interval horizon linearized through the port equations, and its
    first control drives the plant over the same tick. The step reference
    is the constant target seen from the zero initial state. Unconstrained
    Gauss-Newton tracking; constrained variants arrive with the M-B QP
    solver.</p>
  </html>"));
end DoubleIntegratorTrackingMpc;
