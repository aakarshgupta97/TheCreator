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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/install/lib/python2.7/dist-packages:/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build" \
    "/usr/bin/python" \
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex/setup.py" \
    build --build-base "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/install" --install-scripts="/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/install/bin"
