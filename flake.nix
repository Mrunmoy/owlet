{
  description = "owlet — 2-axis pan-tilt head (Raspberry Pi Pico + steppers; USB CDC text protocol)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    flake-utils.url = "github:numtide/flake-utils";

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

  outputs = { self, nixpkgs, flake-utils, pico-sdk-src }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        pythonEnv = pkgs.python3.withPackages (ps: with ps; [
          pyserial
          click
          pillow
        ]);
      in {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            # Raspberry Pi Pico (RP2040, Cortex-M0+).
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
            echo "  pico build:     ./build.py --pico"
            echo "  pico flash:     ./build.py --pico --flash  (board in BOOTSEL, or running owlet)"
            echo "  pc client:      owlet --help"
            echo ""
          '';
        };
      });
}
