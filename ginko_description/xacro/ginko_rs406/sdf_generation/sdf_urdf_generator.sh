# Generate .urdf from .xacro
mkdir -p ./export
mkdir -p ./export/urdf
rosrun xacro xacro --inorder -o  ./export/urdf/ginko_rviz.urdf			"../ginko.xacro" model_mode:=rviz
rosrun xacro xacro --inorder -o  ./export/urdf/ginko_common.urdf		"../ginko.xacro" model_mode:=common
# rosrun xacro xacro --inorder -o  ./export/urdf/ginko_gazebo_part1.urdf	"./ginko.xacro" model_mode:=gazebo_part1
# rosrun xacro xacro --inorder -o  ./export/urdf/ginko_gazebo_part2.urdf	"./ginko.xacro" model_mode:=gazebo_part2

# Generate .sdf from .urdf
mkdir -p ./export/sdf
echo "start common sdf"
gz sdf -p ./export/urdf/ginko_common.urdf       > ./export/sdf/ginko_common.sdf
# echo "start part1 sdf"
# gz sdf -p ./export/urdf/ginko_gazebo_part1.urdf > ./export/sdf/ginko_gazebo_part1.sdf
# echo "start part2 sdf"
# gz sdf -p ./export/urdf/ginko_gazebo_part2.urdf > ./export/sdf/ginko_gazebo_part2.sdf
# Create diff patch
# mkdir -p ./export/patch
# diff -U2  ./export/sdf/ginko_common.sdf ./export/sdf/ginko_gazebo_part1.sdf > ./export/patch/diff_part1.patch
# diff -U2  ./export/sdf/ginko_common.sdf ./export/sdf/ginko_gazebo_part2.sdf > ./export/patch/diff_part2.patch
# Apply patches
# cp ./export/sdf/ginko_common.sdf ./export/sdf/ginko_gazbo.sdf
# patch -u ./export/sdf/ginko_gazbo.sdf < ./export/patch/diff_part1.patch
# patch -u ./export/sdf/ginko_gazbo.sdf < ./export/patch/diff_part2.patch
# Generate URDF
python sdf_close.py
python attach_spring.py

# Copy generated files
mkdir -p ../urdf
mkdir -p ../sdf
cp ./export/urdf/ginko_rviz.urdf ../urdf/ginko_rviz.urdf
cp ./export/sdf/ginko_common_closed_spring.sdf ../sdf/ginko_gazebo.sdf