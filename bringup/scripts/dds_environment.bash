# Source this file in every ROS terminal, including diagnostics and RViz:
# source /path/to/bringup/scripts/dds_environment.bash banana|rasp|notebook|fastdds
# This only changes the current shell's environment; it does not restart nodes.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source dds_environment.bash banana|rasp|notebook|fastdds' >&2
    exit 2
fi

_cbr_dds_environment() {
    local role="${1:-}" profile script_dir
    case "$role" in
        banana|rasp|notebook)
            script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return
            profile="$script_dir/../config/cyclonedds_${role}.xml"
            if [[ ! -r "$profile" ]]; then
                echo "Perfil DDS não encontrado: $profile" >&2
                return 1
            fi
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI="file://$profile"
            ;;
        fastdds)
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            unset CYCLONEDDS_URI
            ;;
        *)
            echo 'Use: source dds_environment.bash banana|rasp|notebook|fastdds' >&2
            return 2
            ;;
    esac
    export ROS_DOMAIN_ID=10
    # Keep all machines discoverable. Peers are managed by the selected XML.
    unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
}

if _cbr_dds_environment "$@"; then
    unset -f _cbr_dds_environment
else
    unset -f _cbr_dds_environment
    return 1
fi
