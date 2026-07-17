{
  description = "CogniPilot Modelica model library development environment";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rumoca.url = "github:CogniPilot/rumoca/149c2ff3939937f5a1345db600830ae4a9a83ca9";
  };

  outputs =
    {
      self,
      flake-utils,
      nixpkgs,
      rumoca,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
        rumocaCli = rumoca.packages.${system}.rumoca;
        rumocaPython = rumoca.packages.${system}.rumoca-python-env;
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
            --replace-fail 'DEFAULT_RUMOCA = None' \
              'DEFAULT_RUMOCA = "${rumocaCli}/bin/rumoca"'
          chmod +x "$out/bin/modelica-models-ci"
          ${python}/bin/python3 -m py_compile "$out/bin/modelica-models-ci"
        '';
        testShell = pkgs.mkShell {
          packages = [
            pkgs.docker-client
            python
            rumocaCli
          ];
        };
        mkQualification =
          name: script:
          pkgs.writeShellApplication {
            inherit name;
            runtimeInputs = [
              python
              rumocaPython
            ];
            text = ''
              models_root="''${MODELICA_MODELS_ROOT:-$PWD}"
              if [ ! -f "$models_root/flake.nix" ] || [ ! -d "$models_root/Vehicles" ]; then
                printf 'error: MODELICA_MODELS_ROOT is not a modelica_models checkout: %s\n' \
                  "$models_root" >&2
                exit 1
              fi
              export MODELICA_MODELS_ROOT="$models_root"
              export PYTHONPATH="${python}/${pkgs.python312.sitePackages}''${PYTHONPATH:+:$PYTHONPATH}"
              exec ${rumocaPython}/bin/python3 ${script} "$@"
            '';
          };
        cubs2Qualification = mkQualification "cubs2-qualification" ./Vehicles/Cubs2/Qualification/run_qualification.py;
        rdd2Qualification = mkQualification "rdd2-qualification" ./Vehicles/Rdd2/Qualification/run_qualification.py;
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
            target,
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
              mkdir -p "$models_root/${output}"
              cd "$models_root"
              exec rumoca compile ${modelFile} \
                --source-root . \
                --model ${modelName} \
                --target ${target} \
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
        rdd2PlantExport = mkModelExport {
          name = "rdd2-export-plant";
          modelFile = "Vehicles/Rdd2/AvionicsPlant.mo";
          modelName = "Vehicles.Rdd2.AvionicsPlant";
          target = "fmi3";
          output = "artifacts/vehicles/rdd2/plant";
        };
      in
      {
        packages.default = ciRunner;
        packages.ci = ciRunner;
        packages.cubs2-qualification = cubs2Qualification;
        packages.rdd2-qualification = rdd2Qualification;
        packages.vehicle-qualification = allVehicleQualification;
        packages.cubs2-export-controller = cubs2ControllerExport;
        packages.cubs2-export-plant = cubs2PlantExport;
        packages.rdd2-export-controller = rdd2ControllerExport;
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
