# Generate .urdf from .xacro
mkdir -p ./tmp
mkdir -p ./tmp/urdf
rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_rviz.urdf			"./ginko.xacro" model_mode:=rviz
rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_common.urdf		"./ginko.xacro" model_mode:=common
# rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_gazebo_part1.urdf	"./ginko.xacro" model_mode:=gazebo_part1
# rosrun xacro xacro --inorder -o  ./tmp/urdf/ginko_gazebo_part2.urdf	"./ginko.xacro" model_mode:=gazebo_part2

# Generate .sdf from .urdf
mkdir -p ./tmp/sdf
echo "start common sdf"
gz sdf -p ./tmp/urdf/ginko_common.urdf       > ./tmp/sdf/ginko_common.sdf
# echo "start part1 sdf"
# gz sdf -p ./tmp/urdf/ginko_gazebo_part1.urdf > ./tmp/sdf/ginko_gazebo_part1.sdf
# echo "start part2 sdf"
# gz sdf -p ./tmp/urdf/ginko_gazebo_part2.urdf > ./tmp/sdf/ginko_gazebo_part2.sdf
# Create diff patch
# mkdir -p ./tmp/patch
# diff -U2  ./tmp/sdf/ginko_common.sdf ./tmp/sdf/ginko_gazebo_part1.sdf > ./tmp/patch/diff_part1.patch
# diff -U2  ./tmp/sdf/ginko_common.sdf ./tmp/sdf/ginko_gazebo_part2.sdf > ./tmp/patch/diff_part2.patch
# Apply patches
# cp ./tmp/sdf/ginko_common.sdf ./tmp/sdf/ginko_gazbo.sdf
# patch -u ./tmp/sdf/ginko_gazbo.sdf < ./tmp/patch/diff_part1.patch
# patch -u ./tmp/sdf/ginko_gazbo.sdf < ./tmp/patch/diff_part2.patch
# Generate URDF
python sdf_close.py
python attach_spring.py

# Copy generated files
mkdir -p ../urdf
mkdir -p ../sdf
cp ./tmp/urdf/ginko_rviz.urdf ../urdf/ginko_rviz.urdf
cp ./tmp/sdf/ginko_common_closed_spring.sdf ../sdf/ginko_gazebo.sdf