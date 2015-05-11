Init.:

Arme init.

roslaunch seneka_ugv_description ugv_bringup.launch robot_ip_r:=192.168.1.11 robot_ip_l:=192.168.1.12
roslaunch seneka_moveit_config move_group.launch

dann auf Home-Verfahren (per UR-GUI, annährend)

roslaunch seneka_pnp seneka_pnp.launch start_state:="collision_free"



Szenario starten:
rosrun seneka_pnp demo.py
