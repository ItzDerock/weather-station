{
  description = "Development environment for LilyGO T-SIM7080G S3";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    arduino-nix.url = "github:bouk/arduino-nix";
    arduino-index = {
      url = "github:bouk/arduino-indexes";
      flake = false; # This repo doesn't have a flake.nix
    };
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    arduino-nix,
    arduino-index,
    ...
  }@attrs:
  let
    packageIndexJson = "${arduino-index}/index/package_index.json";
    packageEsp32IndexJson = "${arduino-index}/index/package_esp32_index.json";
    libraryIndexJson = "${arduino-index}/index/library_index.json";

    # Base overlays from arduino-nix
    baseOverlays = [
      (arduino-nix.overlay)
      (arduino-nix.mkArduinoPackageOverlay packageIndexJson)
      (arduino-nix.mkArduinoPackageOverlay packageEsp32IndexJson)
      (arduino-nix.mkArduinoLibraryOverlay libraryIndexJson)
    ];

    # Overlay to fix TinyGSM build (remains empty, override happens below)
    tinyGsmOverlay = self: super: { };

    # Combine the base overlays and the fix overlay
    overlays = baseOverlays ++ [ tinyGsmOverlay ];

    # UPDATED CDCOnBoot value to lowercase
    boardFQN = "esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,DFUOnBoot=default,FlashMode=qio,FlashSize=16M,MSCOnBoot=default,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc";

  in (flake-utils.lib.eachDefaultSystem (system:
      let
        # Import pkgs with ALL overlays applied
        pkgs = (import nixpkgs) {
          inherit system overlays;
          config.allowUnfree = true;
        };

        # Define the overridden TinyGSM derivation
        tinyGsmLatestOverridden =
          (arduino-nix.latestVersion pkgs.arduinoLibraries.TinyGSM).overrideAttrs
          (oldAttrs: {
            buildPhase = ''
              runHook preBuild
              echo "Skipping default build phase for TinyGSM to avoid platformio dependency"
              runHook postBuild
            '';
            installPhase = oldAttrs.installPhase or ''
              runHook preInstall
              mkdir -p $out/share/arduino/libraries/TinyGSM
              cp -r ./* $out/share/arduino/libraries/TinyGSM/
              runHook postInstall
            '';
          });

        # Define required libraries, using the overridden TinyGSM
        requiredLibraries = with pkgs.arduinoLibraries; [
          tinyGsmLatestOverridden
          (arduino-nix.latestVersion XPowersLib)
          (arduino-nix.latestVersion ArduinoJson)
          (arduino-nix.latestVersion ArduinoHttpClient)
          (arduino-nix.latestVersion TinyGPSPlus)
          (arduino-nix.latestVersion pkgs.arduinoLibraries."Adafruit NeoPixel")
          (arduino-nix.latestVersion pkgs.arduinoLibraries."Soldered PMS7003 Arduino Library")
          (arduino-nix.latestVersion pkgs.arduinoLibraries."Adafruit LTR390 Library")
          (arduino-nix.latestVersion pkgs.arduinoLibraries."Adafruit SHTC3 Library")
          (arduino-nix.latestVersion pkgs.arduinoLibraries."Adafruit LPS2X")
          (arduino-nix.latestVersion pkgs.arduinoLibraries."Adafruit BusIO")
          # *** ADDED LIBRARIES END ***
          # Add others if needed
        ];

        # Define required Arduino core package
        requiredPackages = with pkgs.arduinoPackages; [
          (arduino-nix.latestVersion platforms.esp32.esp32)
        ];

        # *** DEFINE HELPER SCRIPTS PACKAGE ***
        arduinoHelpers = pkgs.stdenv.mkDerivation {
          pname = "arduino-helpers";
          version = "0.1";
          nativeBuildInputs = [ pkgs.makeWrapper ]; # Needed if scripts need wrapping, but not strictly here
          dontUnpack = true; # No src needed

          installPhase = ''
            mkdir -p $out/bin

            # Create ard-compile script
            cat <<-'EOF' > $out/bin/ard-compile
            #!/usr/bin/env bash
            set -euo pipefail # Exit on error, unset var, pipe failure
            sketch_path="$1"
            if [ -z "$sketch_path" ]; then
              echo "Usage: ard-compile <path_to_sketch_folder_or_ino>"
              exit 1
            fi
            if [ -z "''${ARDUINO_BOARD_FQBN:-}" ]; then
              echo "Error: ARDUINO_BOARD_FQBN environment variable is not set."
              exit 1
            fi
            echo "--- Compiling $sketch_path ---"
            arduino-cli compile --fqbn "$ARDUINO_BOARD_FQBN" "$sketch_path"
            echo "--- Compile finished ---"
            EOF

            # Create ard-upload script
            cat <<-'EOF' > $out/bin/ard-upload
            #!/usr/bin/env bash
            set -euo pipefail
            sketch_path="$1"
            serial_port="''${2:-/dev/ttyACM0}" # Default to /dev/ttyACM0 if not provided

            if [ -z "$sketch_path" ]; then
              echo "Usage: ard-upload <path_to_sketch_folder_or_ino> [serial_port]"
              echo "Example: ard-upload ./MySketch /dev/ttyACM1"
              exit 1
            fi
            if [ -z "''${ARDUINO_BOARD_FQBN:-}" ]; then
              echo "Error: ARDUINO_BOARD_FQBN environment variable is not set."
              exit 1
            fi

            echo "--- Uploading $sketch_path to $serial_port ---"
            echo "--- Make sure the board is connected and in bootloader mode if needed (Hold BOOT, press RST, release RST, release BOOT) ---"
            arduino-cli upload --fqbn "$ARDUINO_BOARD_FQBN" -p "$serial_port" "$sketch_path"
            echo "--- Upload finished ---"
            EOF

            # Create ard-flash script
            cat <<-'EOF' > $out/bin/ard-flash
            #!/usr/bin/env bash
            set -euo pipefail
            sketch_path="$1"
            serial_port="''${2:-/dev/ttyACM0}" # Default to /dev/ttyACM0 if not provided

            if [ -z "$sketch_path" ]; then
              echo "Usage: ard-flash <path_to_sketch_folder_or_ino> [serial_port]"
              exit 1
            fi
            # ARDUINO_BOARD_FQBN check happens in ard-compile/ard-upload

            # Call the other scripts, ensuring they are in PATH
            "ard-compile" "$sketch_path" && "ard-upload" "$sketch_path" "$serial_port"
            EOF

            chmod +x $out/bin/*
          '';
        };

      in rec {
        packages.arduino-cli = pkgs.wrapArduinoCLI {
          libraries = requiredLibraries;
          packages = requiredPackages;
        };

        # Development shell
        devShells.default = pkgs.mkShell {
          packages = [
            packages.arduino-cli
            pkgs.esptool
            pkgs.git
            pkgs.python3
            arduinoHelpers
            pkgs.clang # Added clang here as requested previously
          ];

          # *** REMOVED FUNCTION DEFINITIONS, KEPT ENV VAR ***
          shellHook = ''
            export ARDUINO_BOARD_FQBN="${boardFQN}" # Nix interpolation OK here

            echo "-----------------------------------------"
            echo "LilyGO T-SIM7080G S3 Dev Environment"
            echo "-----------------------------------------"
            # Nix interpolation OK here
            echo "Board FQBN set in \$ARDUINO_BOARD_FQBN:" # \$ escapes $ for echo
            echo "  ${boardFQN}"
            echo "Available tools: arduino-cli, esptool, git, python3, clang" # Added clang here
            echo ""
            # *** UPDATED MESSAGE ***
            echo "Helper commands added to PATH:"
            echo "  ard-compile <sketch_path>      # Compiles the sketch"
            echo "  ard-upload <sketch_path> [port] # Uploads (default port: /dev/ttyACM0)"
            echo "  ard-flash <sketch_path> [port]  # Compiles and uploads"
            echo ""
            echo "Example usage:"
            echo "  ard-compile examples/MinimalModemGPSExample"
            echo "  ard-upload examples/MinimalModemGPSExample /dev/ttyACM0"
            echo "  ard-flash examples/MinimalModemGPSExample"
            echo "  esptool.py --port /dev/ttyACM0 flash_id"
            echo "-----------------------------------------"
          '';
        };
      }
    ));
}
