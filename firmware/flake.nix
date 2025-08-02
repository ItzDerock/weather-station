# flake.nix
{
  description = "ESP development environment using nixpkgs-esp-dev";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    # TODO: revert to upstream mirrexagon/nixpkgs-esp-dev when merged
    # https://github.com/mirrexagon/nixpkgs-esp-dev/pull/98
    nixpkgs-esp-dev = {
      url = "github:BastienGermond/nixpkgs-esp-dev/bump-to-5.5";
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

