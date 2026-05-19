docker run -it --rm --net=host --privileged \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e GALLIUM_DRIVER=llvmpipe \
  sowbot:jazzy bash -c "
    Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp &
    sleep 1 &&
    x11vnc -display :99 -nopw -forever -shared -quiet &
    websockify --web /usr/share/novnc 6080 localhost:5900 &
    sleep 1 &&
    export DISPLAY=:99 &&
    gz sim empty.sdf
  "

http://localhost:6080/vnc.html
