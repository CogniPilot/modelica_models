{
  description = "CogniPilot Modelica model library development environment";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    rumoca.url = "github:CogniPilot/rumoca";
    nixpkgs.follows = "rumoca/nixpkgs";
  };

  outputs = { self, flake-utils, nixpkgs, rumoca }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        python = pkgs.python312.withPackages (ps: [
          ps.pip
          ps.pytest
          ps.virtualenv
        ]);
        testShell = pkgs.mkShell {
          inputsFrom = [ rumoca.devShells.${system}.default ];
          packages = [ python ];

          PYTEST_DISABLE_PLUGIN_AUTOLOAD = "1";
        };
      in {
        devShells.default = testShell;
        devShells.ci = testShell;
      });
}
