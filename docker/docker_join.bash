#!/usr/bin/env bash

NAME=locobot_rsa
REPOSITORY="sunfuchou/rsa"
TAG="amd64"
REPO_NAME=LoCoBot-RSA

IMG="${REPOSITORY}:${TAG}"

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo "$containerid"
docker exec -it \
    --privileged \
    -e DISPLAY="${DISPLAY}" \
    -e LINES="$(tput lines)" \
    "${containerid}" \
    bash
xhost -
