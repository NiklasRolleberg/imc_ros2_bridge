# imc_ros2_bridge

Launch node:
ros2 run imc_ros_bridge imc_ros2_bridge.py --ros-args -r __ns:=/robot_namespace -p use_sim_time:=True -p server_ip:=127.0.0.1 -p tcp_port:=7001 -p imc_src:=0x0806