within;
package MediumSlotAsType
  "A member type of a replaceable package collapses to the package slot"

  partial package PartialMedium
    type Pressure = Real;
    type MassFlowRate = Real;
  end PartialMedium;

  partial connector FluidPort
    replaceable package Medium = PartialMedium;
    Medium.Pressure p;
    flow Medium.MassFlowRate m_flow;
  end FluidPort;

  connector VesselPorts extends FluidPort; end VesselPorts;

  connector PlainPort Real p; flow Real m_flow; end PlainPort;

  model ScalarPort "Refused: ET002, `p` has the package slot as its type"
    VesselPorts port;
    Real mb;
  equation
    mb = port.m_flow;
    port.p = 1.0;
  end ScalarPort;

  model ZeroExtentPorts "Refused: ET001, the member itself is gone"
    VesselPorts ports[0];
    Real mb;
  equation
    mb = sum(ports.m_flow);
  end ZeroExtentPorts;

  model ZeroExtentPlainPorts "Compiles: same zero extent, no package slot"
    PlainPort ports[0];
    Real mb;
  equation
    mb = sum(ports.m_flow);
  end ZeroExtentPlainPorts;

  annotation(Documentation(info="<html>
    <p><code>p</code> is declared <code>Medium.Pressure</code>, but the
    compiler reports its type as the package slot
    <code>MediumSlotAsType.FluidPort.Medium</code>: the member selection is
    discarded and the slot is kept in its place. At extent zero the same loss
    presents as the member not existing at all, while the identical zero-extent
    array of a connector with no replaceable package compiles.</p>
  </html>"));
end MediumSlotAsType;
