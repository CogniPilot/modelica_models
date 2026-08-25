{
  description = "Optional pinned tool environment for the CogniPilot Modelica library";

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
          python = pkgs.python312;
          pythonEnvironment = python.withPackages (packages: [
            packages.matplotlib
            packages.numpy
          ]);
          rumocaCli = rumoca.packages.${system}.rumoca;
          rumocaPythonPackage = rumoca.packages.${system}.rumoca-python;
          rumocaRustToolchain =
            pkgs.lib.findFirst (input: pkgs.lib.hasPrefix "rust-nightly-" (input.name or ""))
              (throw "Rumoca Python package has no Rust toolchain build input")
              (rumocaPythonPackage.nativeBuildInputs or [ ]);

          # Rumoca's native extension embeds its build-time Rust toolchain path,
          # but does not load it at runtime. Removing that false reference keeps
          # CI's compiler-binary cache small and tied to the exact Rumoca pin.
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

          # Python is the orchestration source of truth. Nix only installs that
          # command with pinned compiler paths and its Python dependencies.
          modelicaModelsTools = python.pkgs.buildPythonApplication {
            pname = "modelica-models-tools";
            version = "0.1.0";
            pyproject = true;
            src = self;
            build-system = [ python.pkgs.setuptools ];
            dependencies = with python.pkgs; [
              matplotlib
              numpy
            ];
            doCheck = false;
            makeWrapperArgs = [
              "--set-default"
              "MODELICA_MODELS_RUMOCA"
              "${rumocaCli}/bin/rumoca"
              "--set-default"
              "MODELICA_MODELS_PYTHON"
              "${pythonEnvironment}/bin/python3"
              "--prefix"
              "PYTHONPATH"
              ":"
              "${rumocaPythonRuntime}/lib/python3.12/site-packages"
            ];
          };

          developmentShell = pkgs.mkShell (
            {
              packages = [
                modelicaModelsTools
                pythonEnvironment
                rumocaCli
                pkgs.docker-client
                pkgs.gitMinimal
              ]
              ++ pkgs.lib.optionals pkgs.stdenv.isLinux [ openModelicaCli ];
              MODELICA_MODELS_RUMOCA = "${rumocaCli}/bin/rumoca";
              MODELICA_MODELS_DOCKER = "${pkgs.docker-client}/bin/docker";
              MODELICA_MODELS_PYTHON = "${pythonEnvironment}/bin/python3";
              PYTHONPATH = "${rumocaPythonRuntime}/lib/python3.12/site-packages";
            }
            // pkgs.lib.optionalAttrs pkgs.stdenv.isLinux {
              MODELICA_MODELS_OMC = "${openModelicaCli}/bin/omc";
            }
          );
        in
        {
          packages.default = modelicaModelsTools;
          packages.rumoca-cli = rumocaCli;
          packages.rumoca-python-runtime = rumocaPythonRuntime;
          packages.rumoca-runtime = rumocaRuntime;
          apps.default = {
            type = "app";
            program = "${modelicaModelsTools}/bin/modelica-models";
            meta.description = "Run the cross-platform Modelica repository tools";
          };
          devShells.default = developmentShell;
        }
      );
}
