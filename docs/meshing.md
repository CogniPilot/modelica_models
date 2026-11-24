# Generate an external-flow mesh

`Meshing.ExternalFlowMesh` reads a closed ASCII STL and writes the five files in an OpenFOAM `constant/polyMesh` directory. OpenModelica runs the topology algorithm once, during initialization. OpenFOAM can then inspect or solve on the exported mesh.

## First mesh

In OMEdit, open `Meshing/package.mo` and simulate **Meshing → Examples → FlowAroundCube**. The example defines its own geometry and writes `cubeCase/constant/polyMesh` in the simulation working directory. Its two wall layers enclose a one-metre cube and the fluid volume is exactly 63 m³. To mesh another body, use the example below.

## Use your own geometry

With this repository and MSL 4.1.0 on the Modelica library path, save the following as `MeshCase.mo`. Set both paths to absolute paths on your machine. The dimensions below are for a 106.6 mm propeller centred at the origin with its shaft along x. The repository includes a small ASCII test surface at `Meshing/Resources/Geometry/unitCube.stl` for the domain of the cube example. Supply the STL in metres, or set `metresPerUnit=0.001` for millimetres.

```modelica
model MeshCase
  extends Meshing.ExternalFlowMesh(
    surfacePath="/path/to/propeller.stl",
    meshDirectory="/path/to/case/constant/polyMesh",
    lower={-0.3198,-0.2132,-0.2132},
    upper={ 0.6396, 0.2132, 0.2132},
    baseCells={72,32,32},
    levels=2,
    snapFraction=0.2);
end MeshCase;
```

Run this OpenModelica script with `omc mesh.mos` from a build directory containing `MeshCase.mo`.

```modelica
loadModel(Modelica, {"4.1.0"});
loadModel(Meshing);
loadFile("MeshCase.mo");
result := simulate(MeshCase, stopTime=0);
print(result.messages);
getErrorString();
```

Start with a fresh mesh directory for each run. For a case with its usual `system/controlDict`, `fvSchemes` and `fvSolution`, inspect both the standard checks and the extra tests for concavity and cell determinants.

```sh
checkMesh -case /path/to/case -constant
checkMesh -case /path/to/case -constant -allTopology -allGeometry
```

The six domain patches are `xmin`, `xmax`, `ymin`, `ymax`, `zmin` and `zmax`. All solid surfaces share the `body` wall patch. Use these names for the field boundary conditions.

## Other entry points

| Input or task | Entry point |
| --- | --- |
| Triangle coordinates already in memory | `Meshing.generate` |
| Corresponding quadrilateral sections along a twisted wing or blade | `Meshing.loftHexMesh` |
| Linear tetrahedra from a separate geometry pipeline | `Meshing.fromTetrahedra` |
| Prescribed positions at the end of a mesh-motion step | `Meshing.faceSweptVolumes` |

For tetrahedral import, supply points in metres, remap generator node tags to one-based point indices and pass four node indices per cell. Each local boundary ID belongs to the face opposite the corresponding node. Shared faces are discovered automatically. `writePolyMesh` accepts arrays of patch names and patch types indexed by those IDs. A smoke test imported 176 Gmsh HXT tetrahedra, recovered the unit-box volume and passed standard OpenFOAM checks. The extended determinant check flagged 36 cells.

`faceSweptVolumes` returns cubic metres per oriented face over a time interval. Summing by owner and subtracting by neighbour gives the change in each cell's volume. The moving-grid regression checks this identity under a nonuniform bend. Production flutter analysis also requires time-accurate fluid, structural and coupling models.

## Resolution and wall layers

Each refinement level halves the local edge length. The example gives 13.325 mm background edges and 3.33125 mm edges near the surface. To keep a wake region refined, add `refinementBoxes={{{0,-0.06,-0.06},{0.3,0.06,0.06}}}, boxLevels={1}` to the model modifiers. To insert two wall layers, set `layers=2`, give `layerThickness` in metres for their **combined** thickness, and choose a growth ratio moving away from the wall.

For finer resolution specifically around creases, choose a `smoothSurfaceLevel` below `levels`. Triangles incident to edges exceeding `featureAngle` (radians) retain the finest level. This is a dihedral criterion on the supplied tessellation. The function also reports unmatched and nonmanifold edges. Independent edge-constrained snapping and automatic gap sizing remain future work.

Select resolution from the smallest gap or blade thickness you need to resolve. Classification occurs at finest-grid cell centres. Topology repair can thicken a thin trailing edge or close a one-cell gap. Snapping limits vertex displacement to a fraction of the smallest cell edge. Inspect sections through the blade and run the quality checks after changing the snap or layer settings.

## Validation and scope

The analytic cube test checks the exact external volume, number of wall faces, variation in cell size, and volume conservation after adding layers. A separate box refinement test exercises refinement away from a surface. Run the repository assertion suite or simulate `Tests.MeshingTests` directly.

On the exterior surface extracted from the OpenFOAM ONRT propeller wall (2,866 triangles after removing six coincident pairs from the 2,878-triangle extraction), the dimensions above produced 86,009 fluid cells. OpenFOAM v2412 reported maximum non-orthogonality of 25.24° and maximum skewness of 0.549. The standard check passed. The extended face-plane check reported 1,189 concave cells. The original 84,266-cell OpenFOAM reference also triggered that check, with 878 concave cells. Matching a cell count establishes neither blade fidelity nor agreement in thrust or torque.

The native external-flow generator covers a serial Cartesian background, local refinement, bounded surface snapping and optional wall layers around closed solids. The finest-grid classification arrays occupy memory proportional to the entire finest grid. Feature-constrained snapping, independent named surface regions, local layer termination, sliding rotor interfaces and distributed meshing remain outside this implementation.

## Comparison with other meshers

OpenFOAM users will recognize surface refinement, volume refinement, snapping and wall-layer insertion. OpenFOAM also has explicit feature-edge snapping, gap refinement, quality-controlled local layer insertion and parallel meshing. [OpenFOAM snappyHexMesh guide](https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/)

Ansys users can specify local refinement boxes and layer thickness here. Automatic curvature sizing and prescribing a number of cells across a gap are additional capabilities to target. [Ansys sizing controls](https://ansyshelp.ansys.com/public/Views/Secured/corp/v251/en/wb_msh/msh_sf_uniform_curv_prox.html)

The [algorithm review and validation matrix](meshing-algorithms.md) covers propellers, quadrotor wakes, sailboat aerodynamics and wing flutter.
