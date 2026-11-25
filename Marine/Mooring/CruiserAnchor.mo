within Marine.Mooring;
model CruiserAnchor "Rigid anchor and windlass with externally supplied ENU seabed elevations"
  extends PartialCruiserAnchor;
  input Real mountSeabed(unit="m"),anchorSeabed(unit="m");
equation
  bedAtMount=mountSeabed;
  bedAtAnchor=anchorSeabed;
end CruiserAnchor;
