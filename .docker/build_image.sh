#!/usr/bin/env bash

echo -e "Building tbot_mapping:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target bash \
--tag tbot_mapping:latest .