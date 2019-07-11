ginko_obj_path="$(rospack find ginko_description)/meshes/obj/"
ginko_stl_path="$(rospack find ginko_description)/meshes/stl/"
echo $ginko_obj_path
echo $ginko_stl_path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:ginko_obj_path:ginko_stl_path
rosrun xacro xacro --inorder -o  ./urdf_sim/robot_if_rviz.urdf "./urdf_sim/robot_if.xacro" mode:=rviz
rosrun xacro xacro --inorder -o  ./urdf_sim/robot_if_left.urdf "./urdf_sim/robot_if.xacro" mode:=left_arm
rosrun xacro xacro --inorder -o  ./urdf_sim/robot_if_right.urdf "./urdf_sim/robot_if.xacro" mode:=right_arm
gz sdf -p ./urdf_sim/robot_if_rviz.urdf > ./sdf/robot_if_rviz.sdf
gz sdf -p ./urdf_sim/robot_if_left.urdf > ./sdf/robot_if_left.sdf
gz sdf -p ./urdf_sim/robot_if_right.urdf > ./sdf/robot_if_right.sdf
diff -U2  ./sdf/robot_if_rviz.sdf ./sdf/robot_if_left.sdf > ./sdf/robot_if_add_left.patch
diff -U2  ./sdf/robot_if_rviz.sdf ./sdf/robot_if_right.sdf > ./sdf/robot_if_add_right.patch
cp ./sdf/robot_if_rviz.sdf ./sdf/robot_if_right_left.sdf
patch -u ./sdf/robot_if_right_left.sdf < ./sdf/robot_if_add_left.patch
patch -u ./sdf/robot_if_right_left.sdf < ./sdf/robot_if_add_right.patch
rosrun pysdf sdf2urdf.py ./sdf/robot_if_right_left.sdf ./sdf/robot_if_right_left.urdf
sed -i "s@package://PATHTOMESHES@package://ginko_description@g" ./sdf/robot_if_right_left.urdf
sed -i "s@obj@dae@g" ./sdf/robot_if_right_left.urdf 