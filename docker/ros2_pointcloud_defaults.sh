# 大容量点群用Fast DDS設定の既定値（明示設定を優先）
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] &&
   [ -r /ros2_ws/src/docker/fastdds_shm_large_pointcloud.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/docker/fastdds_shm_large_pointcloud.xml
fi

# Bashからの対象bagループ再生に対する先読み既定値
if [ -n "${BASH_VERSION:-}" ]; then
    ros2() {
        local has_target_bag=false
        local has_loop=false
        local has_queue_override=false
        local argument
        if [ "${1:-}" = bag ] && [ "${2:-}" = play ]; then
            for argument in "$@"; do
                case "${argument%/}" in
                    */rosbag2_2026_04_22-19_10_41_transformed|rosbag2_2026_04_22-19_10_41_transformed)
                        has_target_bag=true ;;
                    --loop|-l) has_loop=true ;;
                    --read-ahead-queue-size|--read-ahead-queue-size=*)
                        has_queue_override=true ;;
                esac
            done
        fi
        if [ "$has_target_bag" = true ] && [ "$has_loop" = true ] &&
           [ "$has_queue_override" = false ]; then
            command ros2 "$@" --read-ahead-queue-size 50
        else
            command ros2 "$@"
        fi
    }
fi
