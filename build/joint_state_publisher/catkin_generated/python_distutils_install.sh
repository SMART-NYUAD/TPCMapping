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

echo_and_run cd "/home/john/Projects/PTU-Control/src/joint_state_publisher/joint_state_publisher"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/john/Projects/PTU-Control/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/john/Projects/PTU-Control/install/lib/python3/dist-packages:/home/john/Projects/PTU-Control/build/joint_state_publisher/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/john/Projects/PTU-Control/build/joint_state_publisher" \
    "/usr/bin/python3" \
    "/home/john/Projects/PTU-Control/src/joint_state_publisher/joint_state_publisher/setup.py" \
    egg_info --egg-base /home/john/Projects/PTU-Control/build/joint_state_publisher \
    build --build-base "/home/john/Projects/PTU-Control/build/joint_state_publisher" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/john/Projects/PTU-Control/install" --install-scripts="/home/john/Projects/PTU-Control/install/bin"
