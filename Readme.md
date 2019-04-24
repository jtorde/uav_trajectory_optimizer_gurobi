`source devel/setup.bash`
`roscore`
`rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 map world 10`
`rosrun project main.py`



rosrun project flipper.py __ns:=SQ01s
rosservice call /SQ01s/flip
roslaunch quad_sim quad_sim.launch veh:=SQ num:=01 z:=0
roslaunch acl_demos joy.launch quad:=SQ01s room:=full