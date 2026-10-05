{
  description = "owlet — 2-axis pan-tilt head (ESP32 + servos, or Raspberry Pi Pico + steppers; USB CDC text protocol)";

  inputs = {
    nixpkgs-esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
    # Align nixpkgs to whatever the esp-dev overlay was built against.
    # Avoids "undefined variable" errors caused by attribute renames between
    # nixpkgs releases.
    nixpkgs.follows = "nixpkgs-esp-dev/nixpkgs";
    flake-utils.follows = "nixpkgs-esp-dev/flake-utils";

    # Pico SDK as a source-only input *with submodules*: the nixpkgs pico-sdk
    # package leaves out TinyUSB, and the Pico head's USB port needs it.
    pico-sdk-src = {
      type = "git";
      url = "https://github.com/raspberrypi/pico-sdk.git";
      ref = "refs/tags/2.2.0";
      submodules = true;
      flake = false;
    };
  };

  outputs = { self, nixpkgs, nixpkgs-esp-dev, flake-utils, pico-sdk-src }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nixpkgs-esp-dev.overlays.default ];
          # python3.13-ecdsa-0.19.1 is flagged insecure (CVE-2024-23342 is a
          # timing attack on *signing*; esptool uses ecdsa only for signature
          # verification during flashing). Allowed explicitly so `nix develop`
          # works without --impure.
          config.permittedInsecurePackages = [
            "python3.13-ecdsa-0.19.1"
          ];
        };

        pythonEnv = pkgs.python3.withPackages (ps: with ps; [
          pyserial
          click
          pillow
        ]);
      in {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            # ESP32-S3 is Xtensa; esp-idf-xtensa covers it (replaces the
            # deprecated target-specific esp-idf-esp32s3).
            esp-idf-xtensa

            # Raspberry Pi Pico (RP2040, Cortex-M0+) stepper head.
            gcc-arm-embedded
            picotool
            doxygen

            cmake
            ninja
            gcc

            git
            clang-tools
            pythonEnv

            usbutils
            picocom
          ];

          shellHook = ''
            # The owlet CLI, run from this checkout (no install needed).
            export PATH="$PWD/tools/bin:$PATH"
            export PICO_SDK_PATH=${pico-sdk-src}
            echo "owlet dev shell ready"
            echo "  host build:     ./build.py --host -t"
            echo "  esp build:      ./build.py --esp"
            echo "  esp flash:      ./build.py --esp --flash   (auto-detects the board)"
            echo "  pico build:     ./build.py --pico"
            echo "  pico flash:     ./build.py --pico --flash  (board in BOOTSEL, or running owlet)"
            echo "  pc client:      owlet --help"
            echo ""
          '';
        };
      });
}
