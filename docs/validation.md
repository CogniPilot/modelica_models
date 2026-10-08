# Run validation

Run commands from the repository root. Keep large outputs and caches in an
owned directory under `$HOME/scratch`; use a fresh work directory for each
qualification run.

## Repository checks

```sh
python -m tools.modelica_models_cli check-structure
python -m tools.modelica_models_cli test omc
python -m tools.modelica_models_cli test rumoca
```

See the [build guide](../README.md#testing) for environment setup and the full
suite. Mission assumptions and interpretation belong to the Modelica help
under `Vehicles.Cubs2` and `Vehicles.Rdd2.Test`.

## Visual aiding

Use Python with NumPy, Rumoca 0.10.2 and a C compiler:

```sh
python tools/estimator_comparison/qualify_visual.py \
  --rumoca /path/to/rumoca --cc /path/to/cc \
  --work "$HOME/scratch/modelica_models/visual-check"

python tools/estimator_comparison/qualify_visual_navigation.py \
  --rumoca /path/to/rumoca --cc /path/to/cc \
  --work "$HOME/scratch/modelica_models/visual-navigation"
```

The first checks geometry, corrections and the synthetic landmark camera.
The second compares sequential loose/tight aiding on identical captures across
GPS, GPS-denied and GPS loss/return scenarios. Camera and estimator mathematics
execute as generated Modelica; Python supplies scenarios, noise and offline
RMS/NEES/NIS scoring.

These checks use a fixed known map and do not qualify full uncertain-map SLAM.
See the [comparison tools](../tools/estimator_comparison/README.md) for native
replay prerequisites, campaign inputs and result interpretation.
