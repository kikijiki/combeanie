{
  description = "Reproducible ROS 2 environment for a generic beverage-restocking robot";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs =
    {
      self,
      nixpkgs,
      nix-ros-overlay,
    }:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
      pkgsFor =
        system:
        import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            # Card 048 (launch-teardown SIGSEGV/SIGABRT in parameter_bridge): ros_gz_bridge's
            # parameter_bridge main ends `rclcpp::spin(node); return 0;` with no
            # rclcpp::shutdown(), and rclcpp's Context::shutdown (which runs on the signal
            # handler thread) releases spin() before rcl_logging_fini() destroys the /rosout
            # publisher — so at every SIGINT main() races through node destruction and atexit
            # Fast DDS DomainParticipantFactory destruction while the signal thread is still in
            # DDS, which surfaces as SIGSEGV (use-after-free DataWriter) or SIGABRT
            # (std::system_error from write -> std::terminate). Backtraces and the full
            # mechanism: Backstage evidence/cmbcrash-048/ (Card 048). The one-line call below
            # restores the canonical init -> spin -> shutdown sequence: main blocks on the
            # context init_mutex_ until the signal thread finishes shutdown, then tears down
            # with no other thread in DDS. The defect is still present upstream on
            # gazebosim/ros_gz's ros2 branch (issue creation restricted there); drop this
            # overlay when upstream gains the call.
            (
              final: prev:
              let
                rosPkgs = prev.rosPackages.jazzy;
              in
              {
                rosPackages = prev.rosPackages // {
                  jazzy = rosPkgs // {
                    ros-gz-bridge = rosPkgs.ros-gz-bridge.overrideAttrs (old: {
                      postPatch = (old.postPatch or "") + ''
                        if [ "$(grep -c 'rclcpp::spin(bridge_node);' src/parameter_bridge.cpp)" -ne 1 ]; then
                          echo "Card 048: expected exactly one rclcpp::spin(bridge_node); in parameter_bridge.cpp" >&2
                          exit 1
                        fi
                        sed -i 's|rclcpp::spin(bridge_node);|rclcpp::spin(bridge_node); rclcpp::shutdown();|g' src/parameter_bridge.cpp
                        grep -q 'rclcpp::spin(bridge_node); rclcpp::shutdown();' src/parameter_bridge.cpp
                      '';
                    });
                  };
                };
              }
            )
          ];
        };
      projectSource =
        system:
        let
          pkgs = pkgsFor system;
        in
        pkgs.lib.cleanSourceWith {
          src = ./.;
          filter =
            path: type:
            let
              relative = pkgs.lib.removePrefix (toString ./. + "/") (toString path);
              generated = builtins.any (prefix: pkgs.lib.hasPrefix prefix relative) [
                ".git/"
                "artifacts/"
                "bags/"
                "benchmark-results/"
                "log/"
                "website/node_modules/"
                "website/build/"
                "website/.docusaurus/"
                "ros_ws/build/"
                "ros_ws/build-"
                "ros_ws/install/"
                "ros_ws/install-"
                "ros_ws/log/"
                "ros_ws/log-"
              ];
            in
            !generated;
        };
      foundationRosPackages =
        pkgs: with pkgs.rosPackages.jazzy; [
          ament-cmake
          ament-cmake-core
          ament-cmake-gtest
          ament-cmake-pytest
          ament-lint-auto
          ament-lint-common
          joint-state-publisher
          joint-state-publisher-gui
          launch-ros
          launch-testing
          launch-testing-ament-cmake
          python-cmake-module
          robot-state-publisher
          ros-base
          rviz2
          tf2-ros
          tf2-tools
          ur-description
          urdf
          urdfdom-py
          xacro
        ];
      simulationRosPackages =
        pkgs: with pkgs.rosPackages.jazzy; [
          gz-ros2-control
          ros-gz-bridge
          ros-gz-sim
          ros2-control
          ros2-controllers
        ];
      planningRosPackages =
        pkgs: with pkgs.rosPackages.jazzy; [
          moveit-configs-utils
          moveit-core
          moveit-kinematics
          moveit-planners-ompl
          moveit-ros-move-group
          moveit-ros-planning
          moveit-ros-planning-interface
          moveit-ros-visualization
          moveit-simple-controller-manager
        ];
      rosEnvironment =
        pkgs: packages:
        pkgs.rosPackages.jazzy.buildEnv {
          underlay = true;
          paths = packages;
        };
      mkShell =
        {
          pkgs,
          rosEnv,
          profile,
        }:
        pkgs.mkShell {
          name = "beverage-restocking-jazzy-${profile}";
          # Card 098: mesa joins every profile so GL resolves entirely inside the shell.
          packages = (nativeTools pkgs) ++ [
            pkgs.mesa
            rosEnv
          ];

          ROS_DISTRO = "jazzy";
          RESTOCKER_FLAKE_SHELL = "1";
          RESTOCKER_SHELL_PROFILE = profile;
          COLCON_DEFAULTS_FILE = "${toString ./.}/config/colcon-defaults.yaml";
          # Stdenv's reproducible-builds.sh setup hook seeds `-frandom-seed` from `$out`
          # unless this override is set; `$out` differs per dev shell and rotated every kache
          # cache key across shells (Card 042 finding 2, fixed at source by Card 045).
          NIX_OUTPATH_USED_AS_RANDOM_SEED = "combeanie";

          shellHook = ''
            export RESTOCKER_REPO_ROOT="$PWD"
            echo "beverage-restocking: ROS 2 $ROS_DISTRO ($RESTOCKER_SHELL_PROFILE profile)"
            echo "Run 'just doctor' to validate the workspace."
            # Card 098 (Ogre2 "Unable to create glx fbconfig"): Gazebo's GL plugins reach the GL
            # driver through /run/opengl-driver, which tracks the HOST system's Mesa. After a host
            # update the host driver (Mesa 26.2.3, libLLVM needing GLIBC_2.44) is ABI-skewed
            # against this shell's pinned glibc 2.42, so GLX init dies inside the devshell no
            # matter what the GPU or compositor is doing. Point the loader at this flake's OWN
            # pinned pkgs.mesa (same nixpkgs, same glibc) instead: driver, GLVND vendor json and
            # GBM backend all come from one store path that cannot desync from the host again.
            # LD_LIBRARY_PATH is prepended to, never overwritten — the ROS env already owns it.
            export LD_LIBRARY_PATH=${pkgs.mesa}/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
            export LIBGL_DRIVERS_PATH=${pkgs.mesa}/lib/dri
            export __EGL_VENDOR_LIBRARY_FILENAMES=${pkgs.mesa}/share/glvnd/egl_vendor.d/50_mesa.json
            export GBM_BACKENDS_PATH=${pkgs.mesa}/lib/gbm
          '';
        };
      docsTools = pkgs: [
        pkgs.bun
        pkgs.nodejs
        pkgs.just
        pkgs.python3
      ];
      nativeTools =
        pkgs:
        (docsTools pkgs)
        ++ [
          pkgs.clang-tools
          pkgs.cmake
          pkgs.colcon
          pkgs.cppcheck
          pkgs.direnv
          pkgs.ffmpeg
          pkgs.gcc
          pkgs.gdb
          pkgs.git
          pkgs.graphviz
          pkgs.jq
          pkgs.ninja
          pkgs.nixfmt
          pkgs.pkg-config
          pkgs.protobuf
          pkgs.python3Packages.numpy
          pkgs.python3Packages.pycollada
          pkgs.python3Packages.pyyaml
          pkgs.python3Packages.scipy
          pkgs.python3Packages.trimesh
          pkgs.ruff
          pkgs.shellcheck
          pkgs.yaml-cpp
          pkgs.yq-go
        ];
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = pkgsFor system;
          foundationPackages = foundationRosPackages pkgs;
          foundationRosEnv = rosEnvironment pkgs foundationPackages;
          simulationPackages = simulationRosPackages pkgs;
          simulationRosEnv = rosEnvironment pkgs (foundationPackages ++ simulationPackages);
          baselineRosEnv = rosEnvironment pkgs (
            foundationPackages ++ simulationPackages ++ planningRosPackages pkgs
          );
        in
        {
          docs = pkgs.mkShell {
            name = "combeanie-docs";
            packages = docsTools pkgs;
          };
          foundation = mkShell {
            inherit pkgs;
            rosEnv = foundationRosEnv;
            profile = "foundation";
          };
          default = mkShell {
            inherit pkgs;
            rosEnv = baselineRosEnv;
            profile = "baseline";
          };
          simulation = mkShell {
            inherit pkgs;
            rosEnv = simulationRosEnv;
            profile = "simulation";
          };
          baseline = mkShell {
            inherit pkgs;
            rosEnv = baselineRosEnv;
            profile = "baseline";
          };
        }
      );

      formatter = forAllSystems (system: (pkgsFor system).nixfmt);

      checks = forAllSystems (
        system:
        let
          pkgs = pkgsFor system;
          source = projectSource system;
          rosEnv = rosEnvironment pkgs (
            (foundationRosPackages pkgs) ++ (simulationRosPackages pkgs) ++ (planningRosPackages pkgs)
          );
          futurePackages = (simulationRosPackages pkgs) ++ (planningRosPackages pkgs);
        in
        {
          baseline-package-evaluation =
            assert builtins.all pkgs.lib.isDerivation futurePackages;
            pkgs.runCommand "restocker-baseline-package-evaluation" { } ''
              mkdir -p "$out"
              cat > "$out/packages" <<'EOF'
              gz_ros2_control
              moveit_configs_utils
              moveit_core
              moveit_kinematics
              moveit_planners_ompl
              moveit_ros_move_group
              moveit_ros_planning_interface
              moveit_ros_visualization
              moveit_simple_controller_manager
              ros_gz_bridge
              ros_gz_sim
              ros2_control
              ros2_controllers
              EOF
            '';

          repository-structure =
            pkgs.runCommand "restocker-repository-structure"
              {
                nativeBuildInputs = [ pkgs.python3 ];
              }
              ''
                cp -R ${source} source
                chmod -R u+w source
                cd source
                python3 scripts/check_repository.py
                mkdir -p "$out"
                touch "$out/passed"
              '';

          # `just lint` and `just format-check` were the one developer gate CI did not run. The ament
          # linters registered by each package run inside `colcon test` below, but nothing checked
          # `ruff`, `shellcheck`, `nixfmt` or the repository-wide `ament_uncrustify` pass. This runs the
          # same two recipes a developer runs, not a copy of their contents, so CI and the local command
          # cannot drift apart.
          lint =
            pkgs.runCommand "restocker-lint"
              {
                nativeBuildInputs = (nativeTools pkgs) ++ [ rosEnv ];
                dontWrapQtApps = true;
              }
              ''
                cp -R ${source} source
                chmod -R u+w source
                cd source
                export HOME="$TMPDIR/home"
                mkdir -p "$HOME"
                # `just format-check` executes scripts/format_cpp.bash directly, and a build sandbox has no
                # /usr/bin/env for its shebang to find.
                patchShebangs scripts
                just lint
                just format-check
                mkdir -p "$out"
                touch "$out/passed"
              '';

          ros-workspace = pkgs.stdenvNoCC.mkDerivation {
            pname = "restocker-ros-workspace-check";
            version = "0.1.0";
            src = source;
            nativeBuildInputs = (nativeTools pkgs) ++ [ rosEnv ];
            dontConfigure = true;
            dontWrapQtApps = true;

            buildPhase = ''
              runHook preBuild
              export HOME="$TMPDIR/home"
              mkdir -p "$HOME"
              cd ros_ws
              # These manual colcon commands bypass Nix's CMake configure hook. Bound both
              # scheduling layers: one build package with up to two compiler jobs, or up to
              # two test packages with one CTest job each. Nix's 0 means unlimited; cap it too.
              case "''${NIX_BUILD_CORES:-1}" in
                1) workspace_check_jobs=1 ;;
                *) workspace_check_jobs=2 ;;
              esac
              export CMAKE_BUILD_PARALLEL_LEVEL="$workspace_check_jobs"
              # colcon-cmake otherwise appends native -j<host cores>, overriding CMake's cap.
              export MAKEFLAGS="-j$workspace_check_jobs -l$workspace_check_jobs"
              # RelWithDebInfo is what `config/colcon-defaults.yaml` gives every local build and every
              # benchmark number. Without it colcon leaves CMAKE_BUILD_TYPE empty and this check would test
              # unoptimised binaries, which differ from what anyone else runs and are slow enough to trip the
              # suite's wall-clock budgets.
              colcon build \
                --merge-install \
                --parallel-workers 1 \
                --event-handlers console_cohesion+ \
                --cmake-args -G Ninja -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
              source install/setup.bash

              # Renderer-dependent launch tests are excluded by the ctest label `renderer` in
              # `ros_ws/src/*/CMakeLists.txt`. A Nix build sandbox has no OpenGL implementation,
              # so Ogre2 segfaults on startup, and the software rasteriser that fixes that cannot
              # sustain the sensor rates the RGB-D acceptance tests measure. `just test` runs
              # these tests on the host's real driver. Non-renderer Gazebo physics tests still run
              # here, so this check requires the machine-wide simulator lease. The dense sensor
              # gate (`dense-gate` label, Card 094 / CMB-SPEC-14) is deselected by name too, so
              # the Nix selection's exclusion does not rest on the renderer label alone; the gate
              # runs only through `just dense-gate` on the host.
              echo "--- deselecting ctest labels 'renderer' and 'dense-gate': non-renderer Gazebo physics tests remain selected"
              # Freshness-sensitive coordinator tests must not compete with host-wide worker pools.
              # Explicit CTest serialization also prevents inherited CTEST_PARALLEL_LEVEL fan-out.
              colcon test --merge-install --event-handlers console_cohesion+ \
                --parallel-workers "$workspace_check_jobs" \
                --ctest-args -j1 --label-exclude 'renderer|dense-gate'
              colcon test-result --verbose
              runHook postBuild
            '';

            installPhase = ''
              runHook preInstall
              mkdir -p "$out"
              cp -R install "$out/workspace"
              runHook postInstall
            '';
          };
        }
      );
    };

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };
}
