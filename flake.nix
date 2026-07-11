{
  description = "CogniPilot Modelica model library development environment";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rumoca.url = "github:CogniPilot/rumoca";
  };

  outputs = { self, flake-utils, nixpkgs, rumoca }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        python = pkgs.python3.withPackages (pythonPackages: [
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
              'DEFAULT_RUMOCA = "${rumoca.packages.${system}.rumoca}/bin/rumoca"'
          chmod +x "$out/bin/modelica-models-ci"
          ${python}/bin/python3 -m py_compile "$out/bin/modelica-models-ci"
        '';
        testShell = pkgs.mkShell {
          packages = [
            pkgs.docker-client
            python
            rumoca.packages.${system}.rumoca
          ];
        };
      in {
        packages.default = ciRunner;
        packages.ci = ciRunner;
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
        devShells.default = testShell;
        devShells.ci = testShell;
      });
}
