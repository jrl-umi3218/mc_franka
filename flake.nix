{
  description = "mc-franka: interface between mc_rtc and Franka Emika robots (Panda)";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            flakoboros = {
              enableQt = false;
              rosDistros = [ "jazzy" ];

              overrideAttrs.libfranka =
                { pkgs-final, ... }:
                {
                  src = pkgs-final.fetchgit {
                    url = "https://github.com/jrl-umi3218/libfranka";
                    rev = "f3bbab62bfbbd64a59cf35d427199963630d5506";
                    hash = "sha256-AvAOY9DbzcoOHqvkfL7ADXICX40t+ke+zkunPVcEzVE=";
                    fetchSubmodules = true;
                  };
                  patches = [ ];
                };
              overrideAttrs.mc-franka =
                { pkgs-final, drv-prev, ... }:
                {
                  buildInputs = (drv-prev.buildInputs or []) ++ [ pkgs-final.cli11 ];
                  nativeBuildInputs = [ pkgs-final.cmake ];
                  cmakeFlags = drv-prev.cmakeFlags ++ [ "-DUSE_REALTIME=OFF" ];
                  src = lib.cleanSource ./.;
                };
            };

            # This mc-rtc-superbuild configuration will:
            # - Define named reusable configurations in `configurations`
            # - Use explicit runtime (Nix runtime components) vs devel (local/source overlays)
            # - Generate `${project.name}-<configuration>` and `${project.name}-<configuration>-devel` shells
            #
            # This will also generate a .superbuild/mc_rtc.yaml file containg the suitable mc_rtc configuration
            # Devel dependencies are expected to be installed manually in .superbuild/install
            #
            # As always, individual packages can be overridden using flakoboros
            mc-rtc-superbuild =
              { pkgs, ... }:
              {
                enable = true;
                # TODO: replace this section with your own configuration presets for your project
                configurations = {
                  mc-franka = {
                    extends = [ "minimal" ];
                    runtime = {
                      robots = [
                        pkgs.mc-panda-lirmm
                        pkgs.mc-panda
                      ];

                      apps = [
                        pkgs.mc-rtc-magnum
                      ];
                    };
                    devel = {
                      apps = [
                        pkgs.mc-franka
                      ];
                    };
                  };
                };
              };
          }
        ];
      }
    );
}
