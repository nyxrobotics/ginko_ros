rosrun pysdf sdf2urdf.py ./urdf_sim/robot_gazebo.sdf ./sdf/robot_sim.urdf
gz sdf -p ./sdf/robot_sim.urdf > ./sdf/robot_sim.sdf

rosrun pysdf sdf2urdf.py ./urdf_viz/robot_gazebo.sdf ./sdf/robot_viz.urdf
gz sdf -p ./sdf/robot_sim.urdf > ./sdf/robot_viz.sdf