podman run -it --rm \
  --device /dev/dri \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR:rw,z \
  -v subscriber.py:/subscriber.py:Z \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e QT_QPA_PLATFORM=wayland \
  --security-opt label=type:container_runtime_t \
  --name booster-container booster-container:latest bash

  # --userns=keep-id \
