rosrun xacro xacro --inorder -o ./sdf/robot_sim_spring.urdf ./urdf_sim/robot_gazebo_spring.urdf.xacro
gz sdf -p ./sdf/robot_sim_spring.urdf > ./sdf/robot_sim_spring.sdf

