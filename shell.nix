# Équivalent non-flake: `nix-shell`
(builtins.getFlake (toString ./.)).devShells.${builtins.currentSystem}.default
