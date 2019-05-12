`source devel/setup.bash`
`roscore`
`rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 map world 10`
`rosrun project main.py`





roslaunch quad_sim quad_sim.launch gazebo:=true veh:=SQ num:=01
roslaunch acl_sim sim.launch quad:=SQ01s world_name:="project_good.world"
rviz
rosrun project flipper.py __ns:=SQ01s
rosservice call /SQ01s/takeoff
rosservice call /SQ01s/flip


roslaunch acl_demos joy.launch quad:=SQ01s room:=full