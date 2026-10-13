{
  description = "Cours réalité augmentée (5e année): OpenCV + OpenGL/GLFW/GLEW en C++";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable"; # glfw >= 3.4

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAll = f: nixpkgs.lib.genAttrs systems (s: f nixpkgs.legacyPackages.${s});
    in {
      devShells = forAll (pkgs:
        let
          # highgui avec GTK pour cv::imshow, + ffmpeg/v4l pour la webcam
          opencv = pkgs.opencv.override {
            enableGtk3 = true;
            enableFfmpeg = true;
          };
          libs = with pkgs; [
            opencv
            glew
            glfw
            glm
            nlohmann_json
            libGL
            libGLU
            libx11
            libxrandr
            libxinerama
            libxcursor
            libxi
            wayland
            libxkbcommon
          ];
        in {
          default = pkgs.mkShell {
            nativeBuildInputs = with pkgs; [
              cmake
              ninja
              pkg-config
              gcc
              gdb
              clang-tools # clangd, clang-format
              v4l-utils # debug webcam
            ];
            buildInputs = libs;

            # GLFW charge X11/Wayland/libGL via dlopen
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath libs;
            CMAKE_EXPORT_COMPILE_COMMANDS = "1";

            shellHook = ''
              # nix develop met `-rpath $PWD/outputs/out/lib` dans NIX_LDFLAGS: casse l'édition
              # de liens quand le chemin contient des espaces (ex: "Cours 5e année")
              export NIX_LDFLAGS="''${NIX_LDFLAGS/-rpath $out\/lib/}"
              echo "AR shell: OpenCV $(pkg-config --modversion opencv4), GLFW $(pkg-config --modversion glfw3)"
              echo "  cmake -S . -B build -G Ninja && cmake --build build"
            '';
          };
        });
    };
}
