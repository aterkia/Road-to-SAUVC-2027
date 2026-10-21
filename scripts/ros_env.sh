# Source in every terminal used for this workspace (camera, replay, CLI, tests).
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-LARGE_DATA?max_msg_size=1MB&sockets_size=2MB&non_blocking=true}"
