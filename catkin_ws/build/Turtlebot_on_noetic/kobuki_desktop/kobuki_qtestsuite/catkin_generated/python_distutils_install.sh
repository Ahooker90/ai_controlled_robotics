#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ahooker/catkin_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ahooker/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ahooker/catkin_ws/install/lib/python3/dist-packages:/home/ahooker/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ahooker/catkin_ws/build" \
    "/home/ahooker/anaconda3/bin/python3" \
    "/home/ahooker/catkin_ws/src/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite/setup.py" \
     \
    build --build-base "/home/ahooker/catkin_ws/build/Turtlebot_on_noetic/kobuki_desktop/kobuki_qtestsuite" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ahooker/catkin_ws/install" --install-scripts="/home/ahooker/catkin_ws/install/bin"
