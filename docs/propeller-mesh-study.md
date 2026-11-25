# Iowa port propeller mesh sensitivity

The best complete exterior mesh in this study uses CAD-seeded Delaunay tetrahedra and a Modelica signed-distance cut. The resulting 512,392 fluid cells have one connected propeller wall. Its area differs from the reference by −0.023% and its enclosed volume by −0.299%. It passes native signed-volume, closure and face-pyramid checks. The smallest cell is 1.18 × 10⁻¹⁷ m³. These geometry results do not qualify the mesh for a particular turbulence model or explicit timestep.

![Native CAD-seeded mesh wall and a close-up of the blade tip](propeller-mesh-exterior.png)

The shaft view and the enlarged blade tip trace 131,598 native wall faces, including 59,030 quadrilaterals. The zoom outlines the same faces that bound the fluid cells.

![Section through the actual fluid cells and reference blade](propeller-mesh-section.png)

## Surface geometry and topology

The common input is the repaired, 12,000-triangle, shaft-aligned surface bundled by Ocean Web from the W2025 CAD. The reference area is 150.805 cm² and enclosed volume is 23.658 cm³. All complete meshes use the same outer tank bounds. Surface errors below use the triangulated reference. The finest Cartesian background uses 40 × 64 × 64 logical cells with 1.3 mm axial and 2.0 mm transverse spacing before far-field stretching. The adaptive Cartesian methods spend fewer cells away from the body. The regular level-set method splits each background box into six tetrahedra. [Workshop geometry revision](https://w2025.nl/news/update-of-onrt-shaft-geometry/)

| Method | Fluid cells | Wall shells | Area error | Enclosed volume error | 95th percentile wall → CAD distance |
| --- | ---: | ---: | ---: | ---: | ---: |
| CAD-seeded tetrahedral cut | 512,392 | 1 | −0.023% | −0.299% | 0.037 mm |
| Snapped Cartesian, fine | 112,298 | 1 | +9.40% | +2.22% | 0.625 mm |
| Thin-wall Cartesian, fine | 111,731 | 1 | +27.55% | +10.35% | 1.072 mm |
| Regular level-set cut, fine | 969,173 | 15 | +2.67% | −7.73% | 0.555 mm |
| Fitted prism block, three layers | 36,000 near-wall cells | 1 | 0% to rounding | 0% to rounding | Original facets |

Each nearest-surface test sampled 30,000 area-weighted positions in each direction on the inspection exports. For the CAD-seeded mesh the reverse 95th percentile was 0.036 mm and the largest sampled bidirectional discrepancy was 1.016 mm. This is a sampled maximum. The wall's Euler characteristic was 2, the same as the input. The fine snapped Cartesian mesh had one shell but Euler characteristic −10, indicating additional handles. The regular cut-cell mesh had tiny isolated components and a minimum cell volume of 2.58 × 10⁻²² m³. A visually smooth silhouette, a small total-area error, or positive cell volumes alone can miss these defects.

![Comparison of the original four methods](propeller-mesh-walls.png)

The CAD-seeded background adds samples on both sides of each CAD vertex and triangle centroid and grades the far-field spacing. SciPy/Qhull builds an unconstrained Delaunay background, VTK samples the signed distance, and native Modelica clips the tetrahedra and assembles oriented finite-volume faces. The final wall follows a piecewise-linear distance field. At sample offsets of 70, 140 and 280 μm the wall stayed in one piece in all three runs. The corresponding area errors were +0.028%, −0.023% and +0.457%. This varies one sampling parameter while preserving the source tessellation and far-field recipe.

The separate fitted prism option preserves every input wall triangle. Requested total thicknesses of 25, 50 and 100 μm all passed local cell checks with local thickness limiting. For the nominal 50 μm case, the achieved total ranged from 0.628 to 50 μm. This block still needs a compatible outer core. Attempting to insert three 25 μm-total uniform layers into the CAD-seeded core inverted a cell. The complete mesh and section pictured above omit that failed extrusion. For viscous propeller CFD, published examples use blade-fitted structured blocks and focus resolution in the tip vortex. That is a stronger comparison target for blade loading and wake transport. [Viitanen et al., propeller grid and flow study](https://acris.aalto.fi/ws/portalfiles/portal/27992297/ENG_Viitanen_DDES_of_Wetted_journal_of_Marine_Science_and_Engineering.pdf)

The 140 μm case took 23.6 s for the OpenModelica run that constructed, validated and cached the mesh from an existing sampled background field. Compilation brought that run to 25.5 s. A separate timed invocation of the compiled study took 19.6 s with a cache hit, including file integrity, mesh validation and geometry checks. The binary artifact occupies 65.8 MiB. Background sampling and the CFD solve are outside these timings.

## Flow sensitivity

The GPU study uses the existing Modelica-derived D3Q19 blade-normal slip kernel on an RTX 3090, with 538 rpm, 0.1066 m diameter and a one-revolution startup ramp. Native finite-volume cells and the uniform GPU lattice are different discretizations. To isolate surface representation, each candidate wall was **rasterized back to the same lattice** with the same fractional-occupancy and nearest-normal procedure. Separate runs vary lattice resolution and time step. OpenModelica executed the meshing examples. The reported hydrodynamics are offline Rumoca-kernel results.

| Wall input at J = 0.8, 16 lattice cells/D, 671 steps/turn | Mean Kₜ | Mean Kq | Efficiency | Repeated within tolerance? |
| --- | ---: | ---: | ---: | --- |
| IIHR measurement | 0.3508 | 0.0909 | 0.4912 | Experimental reference |
| Reference CAD | 0.2868 | 0.1289 | 0.2833 | Yes, 12 periods |
| CAD-seeded mesh wall | 0.2836 | 0.1302 | 0.2774 | Yes, 12 periods |

Replacing the CAD with the fitted cut wall changed mean thrust by −1.1% and torque by +1.0% at fixed solver settings. The fitted result still underpredicts IIHR thrust by about 19% and overpredicts torque by about 43%. Surface reconstruction alone explains little of that discrepancy at this operating point. Each of the three regular-grid **medium** walls failed the repeatability test at 32 periods and gave negative last-period thrust. Those runs identify failures of the wall/rasterization/solver combination. Their final values are diagnostics and cannot rank converged propeller efficiencies. [IIHR open-water data](https://simman2020.kr/contents/ONRT.php)

- **Spatial sensitivity.** With the usual lattice-scaled timestep, 12, 16 and 24 cells/D gave Kₜ = 0.1999, 0.2868 and 0.2734. This sequence is nonmonotone. At a common 1,006 steps per turn, the 16 and 24 cells/D runs settled at 0.2610 and 0.2734. The 12 cells/D case failed the repeatability criterion at that timestep.
- **Temporal sensitivity.** Holding the 16 cells/D grid fixed and doubling and quadrupling the usual steps per turn produced runs that did not settle within the 32-period cap. A temporal order of convergence or a grid-convergence index cannot be supported by these data.
- **Wake.** At two diameters downstream, the reference-CAD lattice runs at 12, 16 and 24 cells/D gave mean axial velocity 1.006, 1.104 and 1.090 m/s and mean signed swirl −0.447, −0.418 and −0.433 m/s. A historical steady OpenFOAM calculation gave 1.105 and −0.232 m/s. These are disk means over 12 equal-area annuli and 64 azimuths. The lattice values are final-phase samples. The OpenFOAM field uses an earlier geometry and a different wall treatment. No experimental wake-velocity field was used.

The repeatability test requires three successive turns with less than 2% change in both loads and the sampled near-wake velocity norm. All coefficients above use a full final-period mean. The raw data also include the common seventh period for comparisons at equal physical time. The flow study does not include a no-slip solve directly on the new finite-volume mesh, blade-wall pressure validation or a claim of grid independence.

## Thrust and the mesh-spacing adjoint

A follow-up run at 20 cells per diameter fills the gap in the fixed-timestep lattice study. The four runs use the original CAD, 538 rpm, J = 0.8, 1,006 steps per revolution, the same startup ramp, and the same repeatability test. The last column checks the shaft load alone. It gives the largest relative change between successive turn-mean thrust values over the last four turns, with the newer mean as denominator.

| Cells/D | Spacing (mm) | Turns | Last turn-mean thrust (N) | Loads and wake settled | Largest recent thrust change |
| ---: | ---: | ---: | ---: | :---: | ---: |
| 12 | 8.883 | 32 | 1.029 | No | 0.47% |
| 16 | 6.662 | 16 | 2.706 | Yes | 0.73% |
| 20 | 5.330 | 32 | 2.435 | No | 0.06% |
| 24 | 4.442 | 10 | 2.834 | Yes | 0.18% |

![Fixed-timestep turn-averaged thrust history](propeller-thrust-refinement.png)

The intermediate grid breaks the apparent upward trend between the 16 and 24 cells/D endpoints. The observed changes across grids exceed a 2% thrust tolerance. No mesh cutoff or continuum thrust can be inferred here.

**Reverse-mode check.** At fixed incoming populations, the benchmark's local axial force is `-rho * fraction * (Pxx * gapX + Pxy * gapY + Pxz * gapZ)`. At rest with unit density, occupancy 0.4, shaft angular speed 0.01 in lattice units, z = 2 and Pxx = Pxy = 0.5, Rumoca's steady-state adjoint of the scalar force expression returned 0.01 with respect to occupancy. A centered perturbation of ±0.0001 through the original Modelica collision function returned 0.0100000000000013. This check holds the incoming fluid state fixed. It measures a cell's local partial derivative.

**Coupled reverse-mode check.** An updated local Rumoca build differentiates the original tensor-valued collision function, including concatenation and typed Modelica calls. To test a spacing sensitivity, two interior points at `h = 0.25` and `b = 0.65` define a nonuniform viscous-flow stencil. Moving the first point changes the flow feeding one mixed blade cell. For the steady axial cell load, the adjoint gives **0.0100000 per unit h** and two independent perturbed runs give **0.0100000**. For the load integrated from rest to time 0.2, the adjoint gives **0.00072480128** and two perturbed time integrations give **0.00072480142**. The grid coordinate and the load use the verification model's units. The full-rotor newton-valued objective is a separate test.

**Missing full derivative.** The benchmark constructs the domain from an integer cells/D, quantizes blade occupancy to eight bits and looks up a nearest-facet normal outside the differentiated program. It streams populations, imposes external boundaries and reduces forces in OpenCL. The model exported by Rumoca covers the local collision operator. A full derivative of revolution-mean thrust requires a continuously parameterized mesh/flow coupling and reverse propagation through the flow history or its periodic fixed point. The refinement values above are independent primal runs.

## Computational cost and measured thrust

This sweep uses the original CAD at J = 0.8 and 538 rpm. Every case starts from rest with the same 1,006 steps per turn. The horizontal cost is total lattice cell updates through termination: 144 × (cells/D)³ × steps/turn × completed turns. This includes the time needed to settle. The separate elapsed times are GPU-solver wall times on a shared RTX 3090 and exclude mask generation. The accuracy objective is absolute percentage error in final turn-mean thrust coefficient relative to IIHR (Kₜ = 0.35075). It includes errors from both the numerical discretization and the physical model.

![Observed thrust accuracy against computational work](propeller-thrust-frontier.png)

| Cells/D | Spacing (mm) | Turns | Work (billion cell updates) | Elapsed (s) | Mean thrust (N) | Error vs IIHR | Settled |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | :---: |
| 8 | 13.32 | 26 | 1.93 | 24 | 2.611 | 28.2% | Yes |
| 10 | 10.66 | 32 | 4.64 | 48 | 0.668 | 81.6% | No |
| 12 | 8.88 | 32 | 8.01 | 73 | 1.029 | 71.7% | No |
| 14 | 7.61 | 19 | 7.55 | 71 | 2.912 | 19.9% | Yes |
| 16 | 6.66 | 16 | 9.49 | 80 | 2.706 | 25.6% | Yes |
| 18 | 5.92 | 32 | 27.04 | 291 | 1.772 | 51.3% | No |
| 20 | 5.33 | 32 | 37.09 | 302 | 2.435 | 33.0% | No |
| 22 | 4.85 | 12 | 18.51 | 163 | 3.058 | 15.9% | Yes |
| 24 | 4.44 | 10 | 20.03 | 161 | 2.834 | 22.1% | Yes |

Among runs that passed the existing three-turn load and wake checks, the nondominated grid settings, in ascending work order, are **8, 14, 22 cells/D**. Each buys a smaller experimental thrust discrepancy than every cheaper settled run. Points at the 32-turn limit remain visible as diagnostics. The frontier applies to this operating point, wall treatment, timestep and force objective. A change in measured agreement on these grids does not establish a grid-independent solution.

Regenerate the figure with `python tools/plot_propeller_frontier.py`.

## Reproduce and inspect

Run from the root of `modelica_models`. The `ocean_web` checkout supplies the same input surface and GPU benchmark scripts.

```sh
python ../ocean_web/scripts/benchmarks/onrt-propeller/unpack-surface.py \
  ../ocean_web/viewer/assets/onrt/geometry/cad.bin.gz port.stl
python tools/propeller_mesh_study.py port.stl study --walls
python tools/onrt_mesh_field.py port.stl field.mat --offset 0.00014
```

Simulate `Meshing.Examples.SampledTetStudy(surfacePath="port.stl", fieldPath="field.mat", cacheDirectory="mesh-cache")` at initialization. Its `artifactPath` is the complete reloadable mesh. Use `tools/plot_propeller_wall.py ARTIFACT.mat exterior.png` to reproduce the shaft view and blade tip. Use `tools/plot_propeller_section.py ARTIFACT.mat port.stl section.png` to reproduce the section. The Python background generator uses NumPy, SciPy and VTK. The regression models are `Tests.MeshingTests` and `Tests.MeshingCacheTests`. The standalone geometry sweep accepts `--omc` and `--msl` for an explicit compiler and local standard library.

For the GPU sensitivity runs, generate a mask and normal tensors with `rotor-mask.py WALL.stl mask --normal-tensors normals --resolution 16`. Run `lattice-gpu.py FMU_DIRECTORY mask normals RUN_DIRECTORY --advance-ratio 0.8 --frames 1 --periods 32` using the benchmark's compiled PropellerFlow source bundle. Change `--steps-per-turn` to fix physical timestep across grids or `--time-refinement` on a fixed grid. `check-wake.py CASE_DIRECTORY RUN_DIRECTORY` reads the reference OpenFOAM field through VTK.

Download the [native grid and prism sweep](propeller-mesh-results.csv), [CAD-seeded offset study](propeller-mesh-seeded.csv), [GPU operating points and refinement results](propeller-mesh-flow.csv) and [wake probes](propeller-mesh-wake.csv).
