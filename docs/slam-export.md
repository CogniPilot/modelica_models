# Export SLAM for a browser consumer

`modelica_models` owns the Modelica algorithms; `slam_web` owns browser
presentation and sensor transport. Existing consumers can use generated
flat-name snapshots while the canonical source remains in this repository.

## Generate a snapshot

From the repository root, choose a committed revision and a fresh external
output directory:

```sh
python tools/slam/export_legacy_sources.py --revision HEAD \
  --output "$HOME/scratch/modelica_models/slam-browser-snapshot"
```

The exporter reads committed source only. It writes the consumer's `models/...`
paths plus `slam-provenance.json`, containing the resolved commit and source
hashes. The class mapping is
[`tools/slam/source-manifest.json`](../tools/slam/source-manifest.json).

## Update the consumer

1. Pin a published `modelica_models` commit and use its matching exporter.
2. Load the generated files and check class names, source paths and examples.
3. Run the consumer's runtime and browser tests before removing duplicate sources.
4. Retain the export provenance with the consumer's version information.

Edit algorithms in the canonical Modelica packages, then regenerate the
snapshot. Generated compatibility files are not a second source of truth.
