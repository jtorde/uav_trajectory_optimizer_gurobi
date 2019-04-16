`source devel/setup.bash`
`roscore`
`rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 map world 10`
`rosrun project main.py`