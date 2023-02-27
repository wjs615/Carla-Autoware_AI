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

echo_and_run cd "/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/src/carla_points_map_loader"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/install/lib/python2.7/dist-packages:/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/src/carla_points_map_loader/setup.py" \
     \
    build --build-base "/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/build/carla_points_map_loader" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/install" --install-scripts="/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/install/bin"
