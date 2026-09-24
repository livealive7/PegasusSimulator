#!/usr/bin/env python
"""
| File: 12_multi_backend_standalone.py
| Description: Standalone-app port of the `pegasus.multi_backend_example` Isaac Sim extension.
| Spawns a Multirotor with both an ArduPilot SITL backend and a ROS2 backend (RGB + colorized
| RGBD pointcloud), without needing to open Isaac Sim's UI and click a button.
|
| Run headless (no GUI, for a remote/server box) with:
|   ./python.sh examples/12_multi_backend_standalone.py --headless
|
| Run with the GUI (e.g. for local debugging) with:
|   ./python.sh examples/12_multi_backend_standalone.py
"""

import argparse

parser = argparse.ArgumentParser(description="Spawn a drone with ArduPilot + ROS2 backends")
parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI")
parser.add_argument("--scene-index", type=int, default=9, help="Index into SIMULATION_ENVIRONMENTS to load")
parser.add_argument("--enable-motion-bvh", action="store_true",
                     help="Enable RTX ray-tracing motion BVH (needed for rotary/multi-tick RTX lidars)")
parser.add_argument("--duration", type=float, default=None,
                     help="Auto-close after this many seconds of wall-clock time (for scripted FPS/RTF benchmarks)")
parser.add_argument("--fps-log-interval", type=float, default=1.0,
                     help="How often (wall-clock seconds) to log the current FPS/RTF")
parser.add_argument("--skip-ardupilot", action="store_true",
                     help="Don't attach the ArduPilot backend (it autolaunches SITL via gnome-terminal, "
                          "which isn't available in a headless/server container and will hang the sim "
                          "waiting for a heartbeat that never arrives - useful for pure FPS/RTF benchmarks)")
parser.add_argument("--skip-ros2", action="store_true",
                     help="Don't attach the ROS2 backend (isolates whether its background "
                          "rclpy.spin_once() thread contends for the GIL with the ArduPilot "
                          "backend's own update(), for debugging GPS/MAVLink timing)")
parser.add_argument("--cam-res", type=str, default="320x240", help="Camera resolution WxH")
parser.add_argument("--cam-freq", type=float, default=30.0, help="Camera frequency (Hz)")
parser.add_argument("--render-hz", type=float, default=None, help="Override the world rendering rate (Hz)")
parser.add_argument("--probe-physics", action="store_true", help="Log how physics steps are distributed per render frame")
parser.add_argument("--no-render", action="store_true", help="Step physics with render=False (benchmark: isolates render cost)")
parser.add_argument("--no-depth", action="store_true", help="Disable the camera depth output")
parser.add_argument("--no-writers", action="store_true", help="Keep the camera but skip creating the ROS2 image/depth/camera_info writers (benchmark)")
parser.add_argument("--writers", type=str, default=None, help="Comma list of writers to create: rgb,depth,info (benchmark; others get dummy writers)")
parser.add_argument("--no-camera", action="store_true", help="Don't spawn any camera (benchmark baseline)")
parser.add_argument("--state-div", type=int, default=1, help="ROS2 state_pub_divisor")
parser.add_argument("--sensor-div", type=int, default=1, help="ROS2 sensor_pub_divisor")
parser.add_argument("--pace-fps", type=float, default=None,
                     help="Sleep after each frame so real-time frame pacing doesn't exceed this "
                          "rate (steady, low-jitter cadence) -- testing whether the WebRTC-streamed "
                          "GUI path's lower but more REGULAR frame rate, vs. this script's faster "
                          "but burstier one (500-700ms startup stall, frame_ms swinging ~30%% even "
                          "at steady state per --probe-physics), is why manual GUI flights have never "
                          "hit the cold-liftoff attitude divergence this script's automated runs see "
                          "consistently.")
args, _ = parser.parse_known_args()

# Imports to start Isaac Sim from this script
import carb
import carb.settings
from isaacsim import SimulationApp

import os

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
#
# Unlike when Isaac Sim is launched through the GUI (where the Pegasus extension path is registered
# persistently via the Extension Manager - see docs/source/setup/installation.rst), a standalone
# SimulationApp process starts with a clean extension search path each time, so `pegasus.simulator`
# has to be pointed at and enabled explicitly here, same as `--ext-folder extensions --enable
# pegasus.simulator` in docs/source/setup/developer.rst.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_EXTENSIONS_DIR = os.path.join(_REPO_ROOT, "extensions")

simulation_app = SimulationApp({
    "headless": args.headless,
    "extra_args": ["--ext-folder", _EXTENSIONS_DIR, "--enable", "pegasus.simulator"],
})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import time
import omni.timeline
from isaacsim.core.api.world import World
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, WORLD_SETTINGS, BACKENDS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

try:
    from pegasus.simulator.logic.backends import ROS2Backend
    ROS2_AVAILABLE = True
    if args.no_writers:
        ROS2Backend.update_monocular_camera_data = lambda self, data: None
    if args.writers is not None:
        import sys, types
        _keep = set(args.writers.split(","))
        _names = {"LdrColorSDROS2PublishImage": "rgb", "DistanceToImagePlaneSDROS2PublishImage": "depth", "ROS2PublishCameraInfo": "info"}
        class _Dummy:
            def initialize(self, *a, **k): pass
            def attach(self, *a, **k): pass
        class _Writers:
            def __init__(self, real): self._real = real
            def get(self, name):
                return self._real.get(name) if _names.get(name) in _keep else _Dummy()
        class _Rep:
            def __init__(self, real): self._real = real; self.writers = _Writers(real.writers)
            def __getattr__(self, n): return getattr(self._real, n)
        class _Attr:
            def __init__(self, real): self._real = real
            def set(self, v):
                try: self._real.set(v)
                except Exception: pass
        class _Ctl:
            def __init__(self, real): self._real = real
            def attribute(self, path):
                try: return _Attr(self._real.attribute(path))
                except Exception: return _Attr(None)
            def __getattr__(self, n): return getattr(self._real, n)
        class _OG:
            def __init__(self, real): self._real = real; self.Controller = _Ctl(real.Controller)
            def __getattr__(self, n): return getattr(self._real, n)
        _mod = sys.modules[ROS2Backend.__module__]
        _mod.rep = _Rep(_mod.rep)
        _mod.og = _OG(_mod.og)
except ImportError:
    ROS2_AVAILABLE = False
    carb.log_warn("[MultiBackendStandalone] ROS2 backend not available.")


class PegasusApp:
    """
    Standalone app that spawns a Multirotor with an ArduPilot SITL backend and a ROS2 backend
    (RGB image + colorized RGBD pointcloud), mirroring the pegasus.multi_backend_example extension.
    """

    def __init__(self):
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Optionally enable RTX motion BVH before any lidar/render product is created
        # self._apply_motion_bvh()

        # Apply the world settings used for the ArduPilot/PX4 backends and load the world
        world_settings = dict(WORLD_SETTINGS[BACKENDS["ardupilot"]])
        if args.render_hz:
            world_settings["rendering_dt"] = 1.0 / args.render_hz
        self.pg.set_world_settings(**world_settings)
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load the selected scene
        scene_names = list(SIMULATION_ENVIRONMENTS.keys())
        selected_scene = scene_names[args.scene_index]
        self.pg.load_environment(SIMULATION_ENVIRONMENTS[selected_scene])
        carb.log_warn(f"[MultiBackendStandalone] Scene loaded: {selected_scene}")

        # --- Backend 1: ArduPilot SITL ---
        backends = []
        if not args.skip_ardupilot:
            ardupilot_config = ArduPilotMavlinkBackendConfig({
                "vehicle_id": 0,
                "ardupilot_autolaunch": False,
                "ardupilot_dir": self.pg.ardupilot_path,
                "ardupilot_vehicle_model": self.pg.ardupilot_default_airframe,
                "enable_lockstep": True,
            })
            backends.append(ArduPilotMavlinkBackend(config=ardupilot_config))

        # --- Backend 2: ROS2 ---
        if ROS2_AVAILABLE and not args.skip_ros2:
            backends.append(ROS2Backend(
                vehicle_id=0,
                config={
                    "namespace": "drone",
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": True,
                    "pub_tf": True,
                    "sub_control": True,
                    "state_pub_divisor": args.state_div,
                    "sensor_pub_divisor": args.sensor_div,
                    "pub_clock": True,
                },
            ))

        # --- Assemble and spawn ---
        config_multirotor = MultirotorConfig()
        config_multirotor.backends = backends
        cam_w, cam_h = (int(v) for v in args.cam_res.lower().split("x"))
        config_multirotor.graphical_sensors = [] if args.no_camera else [
            MonocularCamera("camera", config={
                "resolution": (cam_w, cam_h),
                "frequency": args.cam_freq,
                "depth": not args.no_depth,
                "pointcloud": False,
                "pointcloud_stride": 4,
            }),
        ]

        Multirotor(
            "/World/quadrotor",
            ROBOTS["Iris"],
            0,
            (3.0, 0.0, 0.5),
            Rotation.from_euler("XYZ", (0.0, 0.0, 0.0), degrees=True).as_quat(),
            config=config_multirotor,
        )

        carb.log_warn(
            f"[MultiBackendStandalone] Drone spawned, backends: "
            f"{[type(b).__name__ for b in backends]}"
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def _apply_motion_bvh(self):
        """Enable RTX ray-tracing motion BVH so rotary/multi-tick RTX lidars are supported.

        Without this the streaming kit logs "Multi-tick is enabled but motion BVH is not active.
        This is not supported." and drops whole lidar rotations.
        """
        if not args.enable_motion_bvh:
            return
        settings = carb.settings.get_settings()
        settings.set("/rtx/hydra/supportMultiTickRate", True)
        settings.set_bool("/renderer/raytracingMotion/enabled", True)
        settings.set_bool("/renderer/raytracingMotion/enableHydraEngineMasking", True)
        settings.set_string("/renderer/raytracingMotion/enabledForHydraEngines", "0,1,2,3")
        carb.log_warn("[MultiBackendStandalone] motion BVH enabled")

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        Also periodically logs the achieved FPS (wall-clock steps/sec) and RTF (simulated-time /
        wall-clock-time), so `docker exec ... | grep FPS` can be used to benchmark the setup.
        """

        # Start the simulation
        self.timeline.play()

        probe = {"n": 0, "dts": set(), "times": [], "base": 0}
        if args.probe_physics:
            import omni.physx
            probe_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
                lambda dt: (probe.__setitem__("n", probe["n"] + 1), probe["dts"].add(round(dt, 6)), probe["times"].append(time.time()))
            )
            frame_counts = []
            frame_ms = []
            burst_ms = []

        run_start = time.time()
        window_wall_start = run_start
        window_sim_start = self.world.current_time
        window_steps = 0

        pace_interval = (1.0 / args.pace_fps) if args.pace_fps else None

        # The "infinite" loop (or bounded, if --duration was given)
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            n_before = probe["n"]
            t_frame0 = time.time()
            self.world.step(render=not args.no_render)
            window_steps += 1
            if pace_interval is not None:
                remaining = pace_interval - (time.time() - t_frame0)
                if remaining > 0:
                    time.sleep(remaining)
            if args.probe_physics:
                frame_counts.append(probe["n"] - n_before)
                ft = probe["times"][n_before - probe["base"]:] if False else None
                stamps = probe["times"][-(frame_counts[-1]):] if frame_counts[-1] else []
                frame_ms.append((time.time() - t_frame0) * 1e3)
                burst_ms.append((stamps[-1] - stamps[0]) * 1e3 if len(stamps) > 1 else 0.0)
                if len(frame_counts) >= 300:
                    ts = probe["times"]
                    gaps = sorted(b - a for a, b in zip(ts, ts[1:]))
                    carb.log_warn(
                        f"[PROBE] physics steps/frame min={min(frame_counts)} max={max(frame_counts)} "
                        f"mean={sum(frame_counts)/len(frame_counts):.2f} dts={sorted(probe['dts'])} "
                        f"frame_ms={sum(frame_ms)/len(frame_ms):.1f} physics_burst_ms={sum(burst_ms)/len(burst_ms):.1f} "
                        f"gap_ms p50={gaps[len(gaps)//2]*1e3:.2f} p99={gaps[int(len(gaps)*0.99)]*1e3:.2f} max={gaps[-1]*1e3:.2f}"
                    )
                    frame_counts.clear(); probe["times"].clear(); frame_ms.clear(); burst_ms.clear()

            now = time.time()
            elapsed = now - window_wall_start
            if elapsed >= args.fps_log_interval:
                fps = window_steps / elapsed
                sim_elapsed = self.world.current_time - window_sim_start
                rtf = sim_elapsed / elapsed if elapsed > 0 else 0.0
                carb.log_warn(
                    f"[MultiBackendStandalone] FPS={fps:.2f} steps/s | RTF={rtf:.3f} "
                    f"(sim_dt={sim_elapsed:.2f}s / wall_dt={elapsed:.2f}s) | "
                    f"total_wall={now - run_start:.1f}s"
                )
                window_wall_start = now
                window_sim_start = self.world.current_time
                window_steps = 0

            if args.duration is not None and (now - run_start) >= args.duration:
                carb.log_warn(f"[MultiBackendStandalone] --duration={args.duration}s reached, stopping.")
                self.stop_sim = True

        # Cleanup and stop
        carb.log_warn("[MultiBackendStandalone] Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
