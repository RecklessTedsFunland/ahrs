#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/Users/kevin/ros_sandbox/ahrs/devel', type 'exit' to leave"
  . "/Users/kevin/ros_sandbox/ahrs/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/Users/kevin/ros_sandbox/ahrs/devel'"
else
  . "/Users/kevin/ros_sandbox/ahrs/devel/setup.sh"
  exec "$@"
fi
