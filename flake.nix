{
  description = "owlet — 2-axis pan-tilt head (ESP32-S3 + PCA9685 + 2× 180° servos, USB CDC text protocol)";

  inputs = {
    nixpkgs-esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
    # Align nixpkgs to whatever the esp-dev overlay was built against.
    # Avoids "undefined variable" errors caused by attribute renames between
    # nixpkgs releases.
    nixpkgs.follows = "nixpkgs-esp-dev/nixpkgs";
    flake-utils.follows = "nixpkgs-esp-dev/flake-utils";
  };

  outputs = { self, nixpkgs, nixpkgs-esp-dev, flake-utils }:
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
            echo "owlet dev shell ready"
            echo "  host build:     ./build.py --host -t"
            echo "  esp build:      ./build.py --esp"
            echo "  esp flash:      ./build.py --esp --flash   (auto-detects the board)"
            echo "  pc client:      python -m owlet --help   (from tools/)"
            echo ""
          '';
        };
      });
}
