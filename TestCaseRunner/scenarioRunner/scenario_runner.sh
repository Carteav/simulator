#!/bin/bash

set -eu
# set -x

SCENARIO_RUNNER_IMAGE_DEFAULT=lgsvl/simulator-scenarios-runner:simulator-build__2021.2.2
SCENARIOS_DIR=${SCENARIOS_DIR:-$(pwd)}
SCENARIO_RUNNER_IMAGE=${SCENARIO_RUNNER_IMAGE:-${SCENARIO_RUNNER_IMAGE_DEFAULT}}

function load_docker_image {
    IMAGE_NAME=${1}
    TARBALL_DIR=${2}

    TARBALL_NAME=$(echo ${IMAGE_NAME} | tr ':/' '-')

    TARBALL_PATH="${TARBALL_DIR}/${TARBALL_NAME}.tar"

    if ! docker history ${IMAGE_NAME} > /dev/null 2>&1; then
        docker load -i ${TARBALL_PATH}
    fi
}

R=$(dirname $(readlink -f "$0"))

load_docker_image $SCENARIO_RUNNER_IMAGE ${R}/docker

DOCKER_RUN_TTY=--tty
DOCKER_RUN_ARGS=

function prepare_docker_run_args() {
    while [ -n "${1:-}" ]; do
        arg=$1
        shift

        if [[ $arg == *" "* ]]; then
            DOCKER_RUN_ARGS="${DOCKER_RUN_ARGS} '${arg}'"
        else
            DOCKER_RUN_ARGS="${DOCKER_RUN_ARGS} ${arg}"
        fi
    done
}

DOCKER_RUN_ENV_VARS=

# Go through each environment variable with LGSVL__ prefix present,
# then add it to the list of parameters for the `docker run` command
function prepare_docker_run_env_vars() {
    # HACK Temp solution to use old variable
    if [ -z "${LGSVL__MAP:-}" -a -n "${SIMULATOR_MAP:-}" ]; then
        LGSVL__MAP=${SIMULATOR_MAP}
        export LGSVL__MAP
    fi

    echo "Simulation environment:"
    local var
    for var in "${!LGSVL__@}"; do
        printf '%s=%s\n' "$var" "${!var}"
        DOCKER_RUN_ENV_VARS="${DOCKER_RUN_ENV_VARS} -e $var"
    done
}

function run_container() {
    if [ ${#@} == 0 ]; then
        echo "I: Running interactive shell"
        prepare_docker_run_args bash
    else
        prepare_docker_run_args "$@"
    fi

    prepare_docker_run_env_vars

    declare -a MOUNT_SCENARIOS_DIR

    if [ "${SCENARIOS_DIR:-none}" != "none" ]; then
        MOUNT_SCENARIOS_DIR=(--volume "${SCENARIOS_DIR}:/scenarios" --workdir=/scenarios)
    else
        echo "W: SCENARIOS_DIR is not set. scenarios dir is not mounted"
        MOUNT_SCENARIOS_DIR=(--workdir=/)
    fi

    DOCKER_USER=$(id -u):$(id -g)

    echo "Scenario runner image: ${SCENARIO_RUNNER_IMAGE}"

    docker run \
        --rm ${DOCKER_RUN_TTY:-} --interactive \
        --user=${DOCKER_USER} \
        --network=host \
        ${DOCKER_RUN_ENV_VARS} \
        "${MOUNT_SCENARIOS_DIR[@]}" \
        ${SCENARIO_RUNNER_IMAGE} \
        ${DOCKER_RUN_ARGS}
}

function cmd_env() {
    # Go through each LGSVL__ environment variable and print them
    local var
    for var in "${!LGSVL__@}"; do
        printf '%s=%s\n' "$var" "${!var}"
        DOCKER_RUN_ENV_VARS="${DOCKER_RUN_ENV_VARS} -e $var"
    done
    echo "SCENARIOS_DIR=${SCENARIOS_DIR}"
    echo "SCENARIO_RUNNER_IMAGE=${SCENARIO_RUNNER_IMAGE}"
}

function cmd_pull_image() {
        echo "I: Pulling ${SCENARIO_RUNNER_IMAGE}"
        docker pull ${SCENARIO_RUNNER_IMAGE}
}

function cmd_help() {
    CMD=$(basename $0)

cat<<EOF
Usage: ${CMD} help|version|env|bash|run|SHELL_COMMAND [ARGS...]

    help          - Show this message
    version       - Version info
    env           - print usefull environment variables
    bash          - Run container with interactive shell
    run           - Run scenario. This subcommand has own help. See '${CMD} run --help for details'
    SHELL_COMMAND - run shell command inside the container.

Important note: command is launched in a singleshot container so scenarios shall not save any data outside /scenarios folder

ENVIRONMENT VARIABLES:

    * LGSVL__AUTOPILOT_0_HOST - Autopilot bridge hostname or IP. If SVL Simulator is running on a different host than Autopilot, this must be set.
    * LGSVL__AUTOPILOT_0_PORT - Autopilot bridge port
    * LGSVL__AUTOPILOT_0_VEHICLE_CONFIG - Vehicle configuration to be loaded in Dreamview (Capitalization and spacing must match the dropdown in Dreamview)
    * LGSVL__AUTOPILOT_0_VEHICLE_MODULES - Comma-separated list of modules to be enabled in Dreamview. Items must be enclosed by double-quotes and there must not be spaces between the double-quotes and commas. (Capitalization and space must match the sliders in Dreamview)
    * LGSVL__AUTOPILOT_HD_MAP - HD map to be loaded in Dreamview (Capitalization and spacing must match the dropdown in Dreamview)
    * LGSVL__DATE_TIME - Date and time to start simulation at. Time is the local time in the time zone of the map origin. Format 'YYYY-mm-ddTHH:MM:SS'
    * LGSVL__ENVIRONMENT_CLOUDINESS - Value of clouds weather effect, clamped to [0.0, 1.0]
    * LGSVL__ENVIRONMENT_DAMAGE - Value of road damage effect, clamped to [0.0, 1.0]
    * LGSVL__ENVIRONMENT_FOG - Value of fog weather effect, clamped to [0.0, 1.0]
    * LGSVL__ENVIRONMENT_RAIN - Value of rain weather effect, clamped to [0.0, 1.0]
    * LGSVL__ENVIRONMENT_WETNESS - Value of wetness weather effect, clamped to [0.0, 1.0]
    * LGSVL__MAP - Name of map to be loaded in Simulator
    * LGSVL__RANDOM_SEED - Seed used to determine random factors (e.g. NPC type, color, behaviour)
    * LGSVL__SIMULATION_DURATION_SECS - The time length of the simulation [int]
    * LGSVL__SIMULATOR_HOST - SVL Simulator hostname or IP
    * LGSVL__SIMULATOR_PORT - SVL Simulator port
    * LGSVL__SPAWN_BICYCLES - Whether or not to spawn bicycles
    * LGSVL__SPAWN_PEDESTRIANS - Whether or not to spawn pedestrians
    * LGSVL__SPAWN_TRAFFIC - Whether or not to spawn NPC vehicles
    * LGSVL__TIME_OF_DAY - If LGSVL__DATE_TIME is not set, today's date is used and this sets the time of day to start simulation at using the time zone of the map origin, clamped to [0, 24]
    * LGSVL__TIME_STATIC - Whether or not time should remain static (True = time is static, False = time moves forward)
    * LGSVL__VEHICLE_0 - Name of EGO vehicle to be loaded in Simulator
    * SCENARIOS_DIR - Host folder to be mounted as /scenarios inside the container


    '${CMD} env' to list current values
EOF
}

function cmd_version {
    function get_image_label {
        LABEL=$1
        docker inspect --format "{{index .Config.Labels \"com.lgsvlsimulator.scenarios_runner.${LABEL}\"}}" ${SCENARIO_RUNNER_IMAGE}
    }

    echo "Docker image: ${SCENARIO_RUNNER_IMAGE}"
    echo "Build ID:     $(get_image_label 'build_ref')"
}


function test_case_runtime()  {
    echo "Starting TestCase runtime"
    unset DOCKER_RUN_TTY

    SIMULATOR_TC_FILENAME=$(echo ${SIMULATOR_TC_FILENAME} | sed -E 's#^(Python)/##')

    run_container run ${SIMULATOR_TC_FILENAME}
}


if [ -n "${SIMULATOR_TC_RUNTIME:-}" ]; then
    test_case_runtime
else
    case "${1:-}" in
        "pull")
            cmd_pull_image
            ;;
        "env")
            cmd_env
            ;;
        "help"|"--help")
            cmd_help
            ;;
        "version")
            cmd_version
            ;;
        "run")
            shift
            run_container run "$@"
            ;;
        *)
            run_container "$@"
            ;;
    esac
fi
