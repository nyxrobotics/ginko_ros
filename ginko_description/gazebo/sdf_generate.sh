rosrun xacro xacro --inorder -o ./sdf/robot_sim.urdf ./urdf_sim/robot_gazebo.urdf.xacro
gz sdf -p ./sdf/robot_sim.urdf > ./sdf/robot_sim.sdf

rosrun xacro xacro --inorder -o ./sdf/robot_viz.urdf ./urdf_viz/robot_gazebo.urdf.xacro
gz sdf -p ./sdf/robot_sim.urdf > ./sdf/robot_viz.sdf