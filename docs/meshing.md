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

Select resolution from the smallest gap or blade thickness you need to resolve. Classification occurs at finest-grid cell centres. Topology repair keeps manifold one-cell channels and may still thicken a thin edge to repair an ambiguous vertex or edge contact. The snap fraction bounds displacement in units of the finest edge. Snapping backs off locally where adjacent six-quad cells lose face-plane clearance. Pyramid and tetrahedral cells join coarse and fine regions. Inspect sections through the blade and run the quality checks after changing the snap or layer settings.

## Validation and scope

The analytic cube test checks the exact external volume, number of wall faces, variation in cell size, and volume conservation after adding layers. A separate box refinement test exercises refinement away from a surface. Run the repository assertion suite or simulate `Tests.MeshingTests` directly.

The propeller example uses the wall surface extracted from the OpenFOAM ONRT case with six coincident triangle pairs removed, leaving 2,866 triangles. On the full domain above the previous 86,009-cell mesh had 1,189 coarse/fine cells flagged for concavity. The conforming transitions produce 123,824 cells. The following runs used identical geometry and refinement with zero wall layers. Errors are relative to the extracted surface.

| Maximum snap displacement / finest edge | Wall area error | Solid volume error | Extended OpenFOAM v2412 checks |
| ---: | ---: | ---: | --- |
| 0.20 | +6.35% | +18.17% | Pass |
| 0.35 | −1.61% | +7.66% | Pass |
| 0.45 with local backtracking | −5.55% | +1.89% | Pass |

At 0.45 the maximum non-orthogonality was 52.10°, maximum skewness 1.405 and minimum interpolation weight 0.0977. Without backtracking, eight cells failed the extended concavity check. The 84,266-cell OpenFOAM reference had 878 cells flagged for face-plane concavity. A comparison to the original CAD, near-wall resolution and a grid study of thrust, torque and wake profiles are still needed for solver qualification.

Call `Meshing.bodySurfaceMetrics(mesh,triangles)` to obtain reference area, meshed area, reference enclosed volume and meshed enclosed volume. The volume comparison assumes a consistently oriented reference surface, including cavity shells, and a closed solid wall patch. Input and meshed volume use the same units.

The bundled two-layer cube passes the extended checks. A separate two-layer propeller with a shorter domain, 0.328125 mm total layer thickness and 1.2 growth produces inverted face pyramids. The STL export now reports a face and cell number and stops before writing that mesh. Use `python3 tools/check_mesh.py CASE` to gate a generated case on the complete OpenFOAM check result, or add `--docker` for v2412. The native pre-export check covers volume, closure and face orientation. The independent extended check also tests skewness, interpolation weights and other finite-volume quality criteria.

The native external-flow generator covers a serial Cartesian background, local refinement, bounded surface snapping and optional wall layers around closed solids. The finest-grid classification arrays occupy memory proportional to the entire finest grid. Feature-constrained snapping, independent named surface regions, local layer termination, sliding rotor interfaces and distributed meshing remain outside this implementation.

## Comparison with other meshers

OpenFOAM users will recognize surface refinement, volume refinement, snapping and wall-layer insertion. OpenFOAM also has explicit feature-edge snapping, gap refinement, quality-controlled local layer insertion and parallel meshing. [OpenFOAM snappyHexMesh guide](https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/)

Ansys users can specify local refinement boxes and layer thickness here. Automatic curvature sizing and prescribing a number of cells across a gap are additional capabilities to target. [Ansys sizing controls](https://ansyshelp.ansys.com/public/Views/Secured/corp/v251/en/wb_msh/msh_sf_uniform_curv_prox.html)

The [algorithm review and validation matrix](meshing-algorithms.md) covers propellers, quadrotor wakes, sailboat aerodynamics and wing flutter.
