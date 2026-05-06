# modelica_models

Modelica models for CogniPilot projects.

## Layout

- `CogniPilot/`: shared CogniPilot packages.
- `FastDyn/fmu/`: Modelica sources used to generate FastDyn FMUs.

FastDyn looks for this repository beside the FastDyn checkout by default:
`../modelica_models`. Set `MODELICA_MODELS_ROOT` when using a different
checkout path.
