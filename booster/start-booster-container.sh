#!/bin/sh
podman run -it --rm \
  -v /tmp/webots:/tmp/webots/:Z \
  -e USER=$(whoami) \
  --device /dev/input \
  --security-opt label=type:container_runtime_t \
  --name booster-container booster-container:latest
