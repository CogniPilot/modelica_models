within Meshing;
model ExternalFlowMesh "Generate an OpenFOAM case mesh during initialization"
  parameter String surfacePath "Path to a closed triangulated ASCII STL";
  parameter String meshDirectory "New or empty constant/polyMesh directory";
  parameter Real lower[3](each unit="m"),upper[3](each unit="m");
  parameter Integer baseCells[3],levels(min=0)=1;
  parameter Real metresPerUnit=1,snapFraction(min=0,max=0.45)=0.2;
  parameter Integer layers(min=0)=0;
  parameter Real layerThickness(unit="m")=0,growth(min=1)=1.2;
  parameter Real refinementBoxes[:,2,3](each unit="m")=fill(0.0,0,2,3)
    "Optional axis-aligned refinement regions as pairs of lower and upper corners";
  parameter Integer boxLevels[size(refinementBoxes,1)]=fill(levels,size(refinementBoxes,1))
    "Target refinement level inside each region";
  parameter Integer smoothSurfaceLevel(min=0)=levels "Refinement on faces away from sharp edges";
  parameter Real featureAngle(unit="rad")=Modelica.Constants.pi/4 "Dihedral threshold for finest surface refinement";
  final parameter Integer cells(fixed=false);
initial algorithm
  cells:=meshSTL(surfacePath,meshDirectory,lower,upper,baseCells,levels,metresPerUnit,
    snapFraction,layers,layerThickness,growth,refinementBoxes,boxLevels,smoothSurfaceLevel,featureAngle);
end ExternalFlowMesh;
