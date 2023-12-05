#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

uid=$(eval "id -u")
gid=$(eval "id -g")

is_image_build() {
    if [ "$(docker images -q aip_robot_interface_eki/ros:dashing 2> /dev/null)" == "" ]; then
        echo -e "${GREEN}Image not found.${NC}"
        return 1
    else
        echo -e "${RED}Image found. Looks like you've already build the image.${NC}"
        echo -e "${RED}Try running: ./start_docker.sh${NC}"

        while true; do
            read -p 'Do you want to rebuild the image? (y/n) ' yn
            case $yn in
                [Yy] ) return 1;;
                [Nn] ) return 0;;
                * ) echo "Invalid response";;
            esac
        done
    fi
}

build_image() {
    is_image_build
    if [ $? -eq 1 ]; then
        echo -e "${YELLOW}Building image...${NC}"
        docker build \
            --build-arg UID="$uid" \
            --build-arg GID="$gid" \
            -t aip_robot_interface_eki/ros:dashing \
            .
    fi
}

build_image
