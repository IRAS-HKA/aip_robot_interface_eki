#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color


is_image_build() {
    if [ "$(docker images -q aip_robot_interface_eki/ros:dashing 2> /dev/null)" == "" ]; then
        echo -e "${RED}Image not found. Have you build the image?${NC}"
        echo -e "${BLUE}Try running: ./build_docker.sh${NC}"
        return 1
    else
        echo -e "${GREEN}Image found.${NC}"
        return 0
    fi
}
run_docker() {
    is_image_build
    if [ $? -eq 0 ]; then
        echo -e "${YELLOW}Running docker...${NC}"
        xhost + local:root
        docker run \
            --name aip_robot_interface_eki \
            -it \
            --net host \
            -e DISPLAY=$DISPLAY \
            --env-file .env \
            --rm \
            aip_robot_interface_eki/ros:dashing \
            # ros2 run groot Groot
    fi
}

run_docker
