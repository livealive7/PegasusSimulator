# Sourced by scripts/run_standalone*.sh. Exports the env vars Isaac Sim's bundled ROS2 bridge
# needs to actually load rclpy/the RMW implementation. These MUST be set before /isaac-sim/python.sh
# starts - the extension dlopen's librmw_implementation.so (and its dependency libament_index_cpp.so)
# during its own startup, and if LD_LIBRARY_PATH doesn't already point at them at that moment, the
# load fails, the extension aborts its ROS2 setup early, and `import rclpy` fails with
# ModuleNotFoundError even though the rclpy package is sitting right there on disk.
#
# See the extension's own startup log for the canonical instructions this mirrors:
#   docker exec isim-peterson-isaac-sim-1 grep -A6 "Using backup internal ROS2" \
#       /Volume/PegasusSimulator/logs/latest.log

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib"

# The isaac-sim container runs with `--ipc private` (not `--ipc host`), so /dev/shm is NOT shared
# with the host - Fast DDS's default shared-memory transport can therefore discover topics fine
# (discovery is UDP multicast) but silently delivers ZERO messages to/from host-side ROS2 nodes
# (e.g. the host's `ros2 topic hz/echo`), because the SHM segment it tries to use doesn't exist on
# the other side. Force UDP-only so pub/sub actually works across the container boundary.
#
# IMPORTANT: any host-side `ros2` CLI / node that needs to talk to this container's topics must
# export the SAME variable before running, e.g.:
#   export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

# The container image bakes in FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml, but the
# container actually runs as uid 1234 (isaac-sim), which can't read a file under /root - every run
# logs "[XMLPARSER Error] realpath failed Permission denied -> Function loadDefaultXMLFile" because
# of this. Clear it so Fast DDS doesn't try to load a profiles file it has no permission to read.
export FASTRTPS_DEFAULT_PROFILES_FILE=""
