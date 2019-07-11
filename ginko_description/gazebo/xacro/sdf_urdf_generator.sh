# Generate .urdf from .xacro
mkdir -p ./tmp
mkdir -p ./tmp/urdf
rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_rviz.urdf			"./ginko.xacro" model_mode:=rviz
rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_common.urdf		"./ginko.xacro" model_mode:=common
rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_gazebo_part1.urdf	"./ginko.xacro" model_mode:=gazebo_part1
rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_gazebo_part2.urdf	"./ginko.xacro" model_mode:=gazebo_part2

# Generate .sdf from .urdf
mkdir -p ./tmp/sdf
gz sdf -p ./tmp/urdf/ginko_common.urdf       > ./tmp/sdf/ginko_common.sdf
gz sdf -p ./tmp/urdf/ginko_gazebo_part1.urdf > ./tmp/sdf/ginko_gazebo_part1.sdf
gz sdf -p ./tmp/urdf/ginko_gazebo_part2.urdf > ./tmp/sdf/ginko_gazebo_part2.sdf
# Create diff patch
diff -U2  ./tmp/sdf/ginko_common.sdf ./tmp/sdf/ginko_gazebo_part1.sdf > ./patch/diff_part1.patch
diff -U2  ./tmp/sdf/ginko_common.sdf ./tmp/sdf/ginko_gazebo_part2.sdf > ./patch/diff_part2.patch
# Apply patches
cp ./tmp/sdf/ginko_common.sdf ./tmp/sdf/ginko_gazbo.sdf
patch -u ./tmp/sdf/ginko_gazbo.sdf < ./patch/diff_part1.patch
patch -u ./tmp/sdf/ginko_gazbo.sdf < ./patch/diff_part2.patch
# Generate URDF
# rosrun pysdf sdf2urdf.py ./sdf/robot_if_right_left.sdf ./sdf/robot_if_right_left.urdf
# sed -i "s@package://PATHTOMESHES@package://ginko_description@g" ./sdf/robot_if_right_left.urdf
# sed -i "s@obj@dae@g" ./sdf/robot_if_right_left.urdf 
# Copy generated files
cp ./tmp/urdf/ginko_rviz.urdf ../urdf/ginko_rviz.urdf
cp ./tmp/sdf/ginko_gazbo.sdf  ../sdf/ginko_gazbo.sdf