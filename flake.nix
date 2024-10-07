{
  description = "Tutorial for humanoid path planner platform";

  inputs = {
    nixpkgs.url = "github:gepetto/nixpkgs";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ ];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          self',
          pkgs,
          system,
          ...
        }:
        {
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.hpp-tutorial;
            hpp-tutorial = pkgs.python3Packages.hpp-tutorial.overrideAttrs (_: {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./Media
                  ./meshes
                  ./package.xml
                  ./rviz
                  ./script
                  ./src
                  ./srdf
                  ./urdf
                ];
              };
            });
          };
        };
    };
}
