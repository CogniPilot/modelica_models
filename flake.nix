{
  description = "CogniPilot Modelica model library development environment";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    openmodelica.url = "git+https://github.com/jgoppert/OpenModelica?submodules=1&rev=a96aa1a682c463b0fd2d285b486c09a8b7fe496d";
    rumoca.url = "github:CogniPilot/rumoca/1ae998516e12cb31675c5c14afcd803baa2b7f4a";
  };

  outputs =
    {
      self,
      flake-utils,
      nixpkgs,
      openmodelica,
      rumoca,
    }:
    flake-utils.lib.eachSystem
      [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
      ]
      (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
          rumocaCli = rumoca.packages.${system}.rumoca;
          rumocaPython = rumoca.packages.${system}.rumoca-python-env;
          rumocaPythonPackage = rumoca.packages.${system}.rumoca-python;
          rumocaRustToolchain =
            pkgs.lib.findFirst (input: pkgs.lib.hasPrefix "rust-nightly-" (input.name or ""))
              (throw "Rumoca Python package has no Rust toolchain build input")
              (rumocaPythonPackage.nativeBuildInputs or [ ]);
          # The native extension embeds its build-time Rust toolchain path even
          # though it does not load anything from that path at runtime. Remove
          # that false reference so CI can cache the compiler runtime without a
          # multi-gigabyte Rust build closure.
          rumocaPythonRuntime =
            pkgs.runCommand "rumoca-python-runtime-${rumocaPythonPackage.version}"
              {
                nativeBuildInputs = [ pkgs.removeReferencesTo ];
              }
              ''
                cp -a ${rumocaPythonPackage} "$out"
                chmod -R u+w "$out"
                find "$out" -type f -exec \
                  remove-references-to \
                    -t ${rumocaRustToolchain} \
                    -t ${rumocaPythonPackage} \
                    {} +
              '';
          rumocaRuntime = pkgs.symlinkJoin {
            name = "rumoca-runtime-${rumocaCli.version}";
            paths = [
              rumocaCli
              rumocaPythonRuntime
            ];
          };
          openModelicaCli = openmodelica.packages.${system}.default;
          python = pkgs.python312.withPackages (pythonPackages: [
            pythonPackages.matplotlib
          ]);
          ciRunner = pkgs.runCommand "modelica-models-ci" { } ''
            mkdir -p "$out/bin"
            cp ${./tools/ci.py} "$out/bin/modelica-models-ci"
            substituteInPlace "$out/bin/modelica-models-ci" \
              --replace-fail '#!/usr/bin/env python3' '#!${python}/bin/python3' \
              --replace-fail 'DEFAULT_DOCKER = None' \
                'DEFAULT_DOCKER = "${pkgs.docker-client}/bin/docker"' \
              --replace-fail 'DEFAULT_OMC = None' \
                'DEFAULT_OMC = "${openModelicaCli}/bin/omc"' \
              --replace-fail 'DEFAULT_RUMOCA = None' \
                'DEFAULT_RUMOCA = "${rumocaCli}/bin/rumoca"'
            chmod +x "$out/bin/modelica-models-ci"
            ${python}/bin/python3 -m py_compile "$out/bin/modelica-models-ci"
          '';
          rumocaVersionCheck = pkgs.writeShellApplication {
            name = "rumoca-version-check";
            runtimeInputs = [
              rumocaCli
              python
            ];
            text = ''
              cli_version="$(${rumocaCli}/bin/rumoca --version)"
              cli_version="''${cli_version##* }"
              python_version="$(
                PYTHONPATH=${rumocaPythonRuntime}/lib/python3.12/site-packages \
                ${python}/bin/python3 -c \
                  'import rumoca; print(rumoca.version())'
              )"
              if [ "$cli_version" != "$python_version" ]; then
                printf 'error: Rumoca CLI version %s does not match Python version %s\n' \
                  "$cli_version" "$python_version" >&2
                exit 1
              fi

              if cli_identity="$(${rumocaCli}/bin/rumoca build-info 2>/dev/null)"; then
                python_identity="$(
                  PYTHONPATH=${rumocaPythonRuntime}/lib/python3.12/site-packages \
                  ${python}/bin/python3 -c \
                    'import rumoca; print(rumoca.build_identity())'
                )"
                if [ "$cli_identity" != "$python_identity" ]; then
                  printf 'error: Rumoca CLI build %s does not match Python build %s\n' \
                    "$cli_identity" "$python_identity" >&2
                  exit 1
                fi
                printf 'Rumoca CLI/Python build: %s\n' "$cli_identity"
              else
                printf 'Rumoca CLI/Python version: %s\n' "$cli_version"
              fi
            '';
          };
          mkQualification =
            name: script:
            pkgs.writeShellApplication {
              inherit name;
              runtimeInputs = [
                python
                rumocaCli
                rumocaVersionCheck
              ];
              text = ''
                models_root="''${MODELICA_MODELS_ROOT:-$PWD}"
                if [ ! -f "$models_root/flake.nix" ] || [ ! -d "$models_root/Vehicles" ]; then
                  printf 'error: MODELICA_MODELS_ROOT is not a modelica_models checkout: %s\n' \
                    "$models_root" >&2
                  exit 1
                fi
                export MODELICA_MODELS_ROOT="$models_root"
                export PYTHONPATH="${rumocaPythonRuntime}/lib/python3.12/site-packages:${python}/${pkgs.python312.sitePackages}''${PYTHONPATH:+:$PYTHONPATH}"
                rumoca-version-check
                exec ${python}/bin/python3 ${script} "$@"
              '';
            };
          cubs2Qualification = mkQualification "cubs2-qualification" ./Vehicles/Cubs2/Test/run_qualification.py;
          rdd2Qualification = mkQualification "rdd2-qualification" ./Vehicles/Rdd2/Test/run_waypoint_qualification.py;
          mkRdd2MissionPlot =
            name: mode:
            pkgs.writeShellApplication {
              inherit name;
              runtimeInputs = [
                pkgs.gitMinimal
                python
                rumocaPython
              ]
              ++ pkgs.lib.optionals pkgs.stdenv.isLinux [ openModelicaCli ];
              text = ''
                if [ "$(uname -s)" != Linux ]; then
                  printf 'error: RDD2 OpenModelica/Rumoca mission plots are currently Linux-only\n' >&2
                  exit 1
                fi
                models_root="''${MODELICA_MODELS_ROOT:-$PWD}"
                if [ ! -f "$models_root/flake.nix" ] || [ ! -d "$models_root/Vehicles" ]; then
                  printf 'error: MODELICA_MODELS_ROOT is not a modelica_models checkout: %s\n' \
                    "$models_root" >&2
                  exit 1
                fi
                export MODELICA_MODELS_ROOT="$models_root"
                export MODELICAPATH="$models_root"
                export OPENMODELICALIBRARY="$models_root"
                unset PYTHONHOME
                export PYTHONNOUSERSITE=1
                export PYTHONSAFEPATH=1
                export PYTHONPATH="${python}/${pkgs.python312.sitePackages}"
                export RDD2_MISSION_GIT="${pkgs.gitMinimal}/bin/git"
                export RDD2_MISSION_OMC="${openModelicaCli}/bin/omc"
                export RDD2_MISSION_OMC_REVISION="a96aa1a682c463b0fd2d285b486c09a8b7fe496d"
                export RDD2_MISSION_PYTHON="${rumocaPython}/bin/python3"
                export RDD2_MISSION_RUMOCA_REVISION="1ae998516e12cb31675c5c14afcd803baa2b7f4a"
                exec ${rumocaPython}/bin/python3 -P \
                  ${./Vehicles/Rdd2/Test/run_mission_plot.py} \
                  "$@" \
                  ${pkgs.lib.optionalString (mode != null) "--mode ${mode}"}
              '';
            };
          rdd2MissionPlot = mkRdd2MissionPlot "rdd2-mission-plot" null;
          rdd2OpticalMissionPlot = mkRdd2MissionPlot "rdd2-optical-mission-plot" "optical";
          rdd2GpsMissionPlot = mkRdd2MissionPlot "rdd2-gps-mission-plot" "gps";
          trajectoryCompare = pkgs.writeShellApplication {
            name = "trajectory-compare";
            runtimeInputs = [ python ];
            text = ''
              exec ${python}/bin/python3 ${./tools/trajectory_compare.py} "$@"
            '';
          };
          allVehicleQualification = pkgs.writeShellApplication {
            name = "vehicle-qualification";
            runtimeInputs = [
              cubs2Qualification
              rdd2Qualification
            ];
            text = ''
              cubs2-qualification "$@"
              rdd2-qualification
            '';
          };
          mkModelExport =
            {
              name,
              modelFile,
              modelName,
              target ? null,
              emit ? null,
              output,
            }:
            pkgs.writeShellApplication {
              inherit name;
              runtimeInputs = [ rumocaCli ];
              text = ''
                models_root="''${MODELICA_MODELS_ROOT:-$PWD}"
                if [ ! -f "$models_root/${modelFile}" ]; then
                  printf 'error: model source not found: %s/%s\n' \
                    "$models_root" ${modelFile} >&2
                  exit 1
                fi
                mkdir -p "$(dirname "$models_root/${output}")"
                cd "$models_root"
                exec rumoca compile ${modelFile} \
                  --source-root . \
                  --model ${modelName} \
                  ${if target != null then "--target ${target}" else "--emit ${emit}"} \
                  --output ${output} \
                  "$@"
              '';
            };
          cubs2ControllerExport = mkModelExport {
            name = "cubs2-export-controller";
            modelFile = "Vehicles/Cubs2/OuterLoop.mo";
            modelName = "Vehicles.Cubs2.OuterLoop";
            target = "galec-production";
            output = "artifacts/vehicles/cubs2/controller";
          };
          cubs2PlantExport = mkModelExport {
            name = "cubs2-export-plant";
            modelFile = "Vehicles/Cubs2/AvionicsPlant.mo";
            modelName = "Vehicles.Cubs2.AvionicsPlant";
            target = "fmi3";
            output = "artifacts/vehicles/cubs2/plant";
          };
          rdd2ControllerExport = mkModelExport {
            name = "rdd2-export-controller";
            modelFile = "Vehicles/Rdd2/Controller.mo";
            modelName = "Vehicles.Rdd2.Controller";
            target = "galec-production";
            output = "artifacts/vehicles/rdd2/controller";
          };
          rdd2EstimatorExport = mkModelExport {
            name = "rdd2-export-estimator";
            modelFile = "Vehicles/Rdd2/NavigationEstimator.mo";
            modelName = "Vehicles.Rdd2.NavigationEstimator";
            target = "galec-production";
            output = "artifacts/vehicles/rdd2/estimator";
          };
          rdd2PlantExport = mkModelExport {
            name = "rdd2-export-plant";
            modelFile = "Vehicles/Rdd2/Plant.mo";
            modelName = "Vehicles.Rdd2.Plant";
            target = "fmi3";
            output = "artifacts/vehicles/rdd2/plant";
          };
          testShell = pkgs.mkShell {
            packages = [
              pkgs.docker-client
              python
              rumocaCli
              ciRunner
              cubs2Qualification
              rdd2Qualification
              rdd2MissionPlot
              rdd2OpticalMissionPlot
              rdd2GpsMissionPlot
              allVehicleQualification
              rumocaVersionCheck
              trajectoryCompare
            ]
            ++ pkgs.lib.optionals pkgs.stdenv.isLinux [ openModelicaCli ];
            shellHook = ''
              if [ -z "''${MODELICA_MODELS_ROOT:-}" ] \
                && [ -f "$PWD/flake.nix" ] \
                && [ -d "$PWD/Vehicles" ]; then
                export MODELICA_MODELS_ROOT="$PWD"
              fi

              if [ -n "''${MODELICA_MODELS_ROOT:-}" ]; then
                export MODELICAPATH="$MODELICA_MODELS_ROOT''${MODELICAPATH:+:$MODELICAPATH}"
                export OPENMODELICALIBRARY="$MODELICA_MODELS_ROOT''${OPENMODELICALIBRARY:+:$OPENMODELICALIBRARY}"
              fi

              printf '%s\n' \
                'Vehicle qualification commands:' \
                '  vehicle-qualification  Run every named vehicle qualification' \
                '  cubs2-qualification    Run the CUBS2 qualification' \
                '  rdd2-qualification     Run the RDD2 qualification' \
                '  rdd2-mission-plot      Report a named RDD2 optical/GPS trace' \
                '  rdd2-optical-mission-plot  Report the optical-flow mission trace' \
                '  rdd2-gps-mission-plot      Report the GPS mission trace' \
                '  rumoca-version-check   Verify CLI/Python compiler identity' \
                '  trajectory-compare     Compare canonical trajectory logs'
            '';
          };
        in
        {
          packages.default = ciRunner;
          packages.ci = ciRunner;
          packages.rumoca-cli = rumocaCli;
          packages.rumoca-python-runtime = rumocaPythonRuntime;
          packages.rumoca-runtime = rumocaRuntime;
          packages.cubs2-qualification = cubs2Qualification;
          packages.rdd2-qualification = rdd2Qualification;
          packages.rdd2-mission-plot = rdd2MissionPlot;
          packages.rdd2-optical-mission-plot = rdd2OpticalMissionPlot;
          packages.rdd2-gps-mission-plot = rdd2GpsMissionPlot;
          packages.rumoca-version-check = rumocaVersionCheck;
          packages.trajectory-compare = trajectoryCompare;
          packages.vehicle-qualification = allVehicleQualification;
          packages.cubs2-export-controller = cubs2ControllerExport;
          packages.cubs2-export-plant = cubs2PlantExport;
          packages.rdd2-export-controller = rdd2ControllerExport;
          packages.rdd2-export-estimator = rdd2EstimatorExport;
          packages.rdd2-export-plant = rdd2PlantExport;
          apps.default = {
            type = "app";
            program = "${ciRunner}/bin/modelica-models-ci";
            meta.description = "Run the Modelica compiler and plotting checks";
          };
          apps.ci = {
            type = "app";
            program = "${ciRunner}/bin/modelica-models-ci";
            meta.description = "Run the Modelica compiler and plotting checks";
          };
          apps.cubs2-qualification = {
            type = "app";
            program = "${cubs2Qualification}/bin/cubs2-qualification";
            meta.description = "Run CUBS2 model-level flight qualification";
          };
          apps.rdd2-qualification = {
            type = "app";
            program = "${rdd2Qualification}/bin/rdd2-qualification";
            meta.description = "Run RDD2 model-level flight qualification";
          };
          apps.rdd2-mission-plot = {
            type = "app";
            program = "${rdd2MissionPlot}/bin/rdd2-mission-plot";
            meta.description = "Report an explicit RDD2 optical-flow or GPS mission trace";
          };
          apps.rdd2-optical-mission-plot = {
            type = "app";
            program = "${rdd2OpticalMissionPlot}/bin/rdd2-optical-mission-plot";
            meta.description = "Report the RDD2 optical-flow waypoint mission trace";
          };
          apps.rdd2-gps-mission-plot = {
            type = "app";
            program = "${rdd2GpsMissionPlot}/bin/rdd2-gps-mission-plot";
            meta.description = "Report the RDD2 GPS waypoint mission trace";
          };
          apps.rumoca-version-check = {
            type = "app";
            program = "${rumocaVersionCheck}/bin/rumoca-version-check";
            meta.description = "Verify Rumoca CLI and Python compiler identity";
          };
          apps.trajectory-compare = {
            type = "app";
            program = "${trajectoryCompare}/bin/trajectory-compare";
            meta.description = "Compare canonical vehicle trajectory logs";
          };
          apps.vehicle-qualification = {
            type = "app";
            program = "${allVehicleQualification}/bin/vehicle-qualification";
            meta.description = "Run all named-vehicle model qualification";
          };
          apps.cubs2-export-controller = {
            type = "app";
            program = "${cubs2ControllerExport}/bin/cubs2-export-controller";
            meta.description = "Export the CUBS2 controller as eFMI Production Code";
          };
          apps.cubs2-export-plant = {
            type = "app";
            program = "${cubs2PlantExport}/bin/cubs2-export-plant";
            meta.description = "Export the CUBS2 avionics plant as FMI 3";
          };
          apps.rdd2-export-controller = {
            type = "app";
            program = "${rdd2ControllerExport}/bin/rdd2-export-controller";
            meta.description = "Export the RDD2 controller as eFMI Production Code";
          };
          apps.rdd2-export-estimator = {
            type = "app";
            program = "${rdd2EstimatorExport}/bin/rdd2-export-estimator";
            meta.description = "Export the RDD2 attitude estimator as eFMI Production Code";
          };
          apps.rdd2-export-plant = {
            type = "app";
            program = "${rdd2PlantExport}/bin/rdd2-export-plant";
            meta.description = "Export the RDD2 avionics plant as FMI 3";
          };
          devShells.default = testShell;
          devShells.ci = testShell;
        }
      );
}
