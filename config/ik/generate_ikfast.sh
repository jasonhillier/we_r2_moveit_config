#!/bin/bash

echo "Converting URDF..."
xacro ../we_r2_arm.urdf.xacro > urdf.tmp
rosrun collada_urdf urdf_to_collada urdf.tmp we_r2_arm.dae

echo "Getting links..."
docker run --rm --env PYTHONPATH=/usr/local/lib/python2.7/dist-packages -v $(pwd):/out hamzamerzic/openrave /bin/bash -c "cd /out; openrave-robot.py we_r2_arm.dae --info links"

echo "Generating cpp file..."
docker run --rm --env PYTHONPATH=/usr/local/lib/python2.7/dist-packages -v $(pwd):/out hamzamerzic/openrave /bin/bash -c "cd /out; python /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_/ikfast.py --robot=we_r2_arm.dae --iktype=transform6d --baselink=0 --eelink=6 --savefile=we_r2_arm_ik.cpp"
