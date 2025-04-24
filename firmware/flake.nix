# flake.nix
{
  description = "ESP development environment using nixpkgs-esp-dev";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    nixpkgs-esp-dev = {
      url = "github:mirrexagon/nixpkgs-esp-dev";
      # inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, nixpkgs-esp-dev, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nixpkgs-esp-dev.overlays.default
          ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "esp-project";

          buildInputs = with pkgs; [
            esp-idf-full
          ];
        };
      });
}

