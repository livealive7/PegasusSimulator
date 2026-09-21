"""
| File: extension.py
| Description: Minimal Isaac Sim extension example.
|              One window, one button. Clicking it spawns a Multirotor
|              with both an ArduPilot SITL backend and a ROS2 backend.
|
| Assumes the Pegasus Simulator extension is enabled and Isaac Sim's
| world is already up (same prerequisite as running this in the
| Script Editor).
"""

import asyncio
import time
from scipy.spatial.transform import Rotation

import carb
import carb.settings
import omni.ext
import omni.ui as ui
import omni.kit.app

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, WORLD_SETTINGS, BACKENDS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.backends import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar

try:
    from pegasus.simulator.logic.backends import ROS2Backend
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    carb.log_warn("[MultiBackendExample] ROS2 backend not available.")


class MultiBackendExampleExtension(omni.ext.IExt):
    """
    Minimal working extension example.

    Isaac Sim's extension system calls on_startup() automatically when
    enabled, and on_shutdown() when disabled/reloaded.
    """
    
    # 實驗 B: 開啟 RTX ray-tracing motion BVH。Example_Rotary 這類旋轉式 lidar 用 multi-tick
    # 累積一整圈,而 multi-tick 需要 motion BVH 才「supported」;streaming kit 預設沒帶到這組
    # 設定,才會有 "Multi-tick is enabled but motion BVH is not active. This is not supported."
    # 的 warning,並導致整圈被丟(lidar stall)。設 True 在 spawn 前把它打開。
    # 註:這是 renderer 設定,runtime 設定「通常」會生效,但最保險是寫進 kit 啟動參數
    # (--/renderer/raytracingMotion/enabled=true 等);若 runtime 設了 warning 仍在,就要改走啟動參數。
    ENABLE_MOTION_BVH = False

    def on_startup(self, ext_id):
        carb.log_warn("[MultiBackendExample] Extension started")

        # 實驗 B: 盡早開啟 motion BVH(在建 lidar 之前)
        self._apply_motion_bvh()

        self._window = ui.Window("Multi Backend Spawner", width=300, height=150)

        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label(
                    "Spawn a drone with ArduPilot + ROS2 backends",
                    word_wrap=True,
                )
                ui.Button(
                    "Spawn Drone (ArduPilot + ROS2)",
                    height=40,
                    clicked_fn=self._on_spawn_button_clicked,
                )
                
    # ------------------------------------------------------------------
    # 實驗 B: RTX motion BVH
    # ------------------------------------------------------------------
    def _apply_motion_bvh(self):
        """Enable RTX ray-tracing motion BVH so rotary/multi-tick RTX lidars are supported.

        Without this the streaming kit logs "Multi-tick is enabled but motion BVH is not active.
        This is not supported." and drops whole lidar rotations. These are renderer settings; the
        robust place is the kit launch args, but setting them here before the lidar/render product
        is created usually takes effect too.
        """
        if not self.ENABLE_MOTION_BVH:
            return
        settings = carb.settings.get_settings()
        settings.set("/rtx/hydra/supportMultiTickRate", True)
        settings.set_bool("/renderer/raytracingMotion/enabled", True)
        settings.set_bool("/renderer/raytracingMotion/enableHydraEngineMasking", True)
        settings.set_string("/renderer/raytracingMotion/enabledForHydraEngines", "0,1,2,3")
        carb.log_warn(
            "[MultiBackendExample] motion BVH enabled "
            f"(/renderer/raytracingMotion/enabled={settings.get('/renderer/raytracingMotion/enabled')})"
        )

    def on_shutdown(self):
        carb.log_warn("[MultiBackendExample] Extension shutting down")
        self._window = None
        self._cleanup_old_vehicles()
        try:
            PegasusInterface().clear_scene()
        except Exception as e:
            carb.log_warn(f"[MultiBackendExample] Exception during clear_scene: {e}")

    def _cleanup_old_vehicles(self):
        """
        Clean up old vehicles' backends + physics callback subscriptions,
        so a stale vehicle doesn't keep getting update() called on every
        physics step and crash trying to access a prim path that no
        longer exists on the new stage.

        Note: ROS2Backend.stop() in the source only resets the rotor
        reference values - it never cleans up the rclpy node/publishers/
        subscribers. The node stays alive and the topics stay around
        even after the extension is unloaded. This manually calls
        node.destroy_node() to cover what ROS2Backend.stop() misses.
        """
        vehicle_manager = VehicleManager()
        if len(vehicle_manager.vehicles) == 0:
            return

        carb.log_warn(
            f"[MultiBackendExample] Found {len(vehicle_manager.vehicles)} old vehicle(s), cleaning up"
        )
        for vehicle in vehicle_manager.vehicles.values():
            for backend in getattr(vehicle, "_backends", []):
                if hasattr(backend, "stop"):
                    try:
                        backend.stop()
                    except Exception as e:
                        carb.log_warn(
                            f"[MultiBackendExample] Exception in backend.stop(): {e}"
                        )
                # ROS2Backend-specific cleanup: node destruction that stop() misses
                if hasattr(backend, "node"):
                    try:
                        backend.node.destroy_node()
                        carb.log_warn(
                            f"[MultiBackendExample] Destroyed ROS2 node: {backend.node.get_name()}"
                        )
                    except Exception as e:
                        carb.log_warn(
                            f"[MultiBackendExample] Exception in node.destroy_node(): {e}"
                        )
            if hasattr(vehicle, "_physics_step_subs"):
                vehicle._physics_step_subs.clear()
        vehicle_manager.remove_all_vehicles()

    def _on_spawn_button_clicked(self):
        """Button callback, dispatched into async so the UI thread doesn't block"""
        asyncio.ensure_future(self._spawn_drone_async())

    async def _spawn_drone_async(self):
        pg = PegasusInterface()

        # --- Clean up old vehicles first (ROS2 nodes, ArduPilot SITL process) ---
        self._cleanup_old_vehicles()

        # Re-apply motion BVH before (re)creating the lidar, in case the renderer context
        # was reset by a previous load_environment / clear_scene.
        # self._apply_motion_bvh()

        # --- Load scene, same logic as ui_delegate.py's on_load_scene() ---
        # Fixed to pick scene index 9 here; change to any key in SIMULATION_ENVIRONMENTS as needed
        scene_names = list(SIMULATION_ENVIRONMENTS.keys())
        selected_scene = scene_names[9]

        pg.set_world_settings(**WORLD_SETTINGS[BACKENDS["ardupilot"]])
        await pg.load_environment_async(
            SIMULATION_ENVIRONMENTS[selected_scene], force_clear=True
        )

        carb.log_warn(f"[MultiBackendExample] Scene loaded: {selected_scene}")

        # --- Key point: load_environment_async opens a brand new stage
        #     internally, which invalidates the pg.world Python reference.
        #     Must re-fetch the latest world from PegasusInterface() -
        #     can't reuse the old pg variable's world reference ---
        pg = PegasusInterface()

        # Make sure the physics context is initialized
        if hasattr(pg.world, "_physics_context") is False:
            await pg.world.initialize_simulation_context_async()

        # --- Backend 1: ArduPilot SITL ---
        ardupilot_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": 0,
            "ardupilot_autolaunch": False,
            "ardupilot_dir": pg.ardupilot_path,
            "ardupilot_vehicle_model": pg.ardupilot_default_airframe
        })
        ardupilot_backend = ArduPilotMavlinkBackend(config=ardupilot_config)

        backends = [ardupilot_backend]

        # --- Backend 2: ROS2 ---
        if ROS2_AVAILABLE:
            ros2_backend = ROS2Backend(
                vehicle_id=0,
                config={
                    "namespace": "drone",
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": True,
                    "pub_tf": True,
                    "sub_control": True,
                    "state_pub_divisor": 1,
                    "sensor_pub_divisor": 1,
                },
            )
            backends.append(ros2_backend)

        # --- Assemble and spawn ---
        config_multirotor = MultirotorConfig()
        # backends = []
        config_multirotor.backends = backends
        config_multirotor.graphical_sensors = [
            MonocularCamera("camera", config={
                "resolution": (640, 480),
                "frequency": 30.0,
            }),
            # Lidar("lidar", config={
            #     "frequency": 10.0,
            #     "sensor_configuration": "Simple_Example_Solid_State",
            # })
            # Lidar("lidar", config={
            #     "frequency": 10.0,
            #     "sensor_configuration": "Example_Rotary",
            # })
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
            f"[MultiBackendExample] Drone spawned, backends: "
            f"{[type(b).__name__ for b in backends]}"
        )