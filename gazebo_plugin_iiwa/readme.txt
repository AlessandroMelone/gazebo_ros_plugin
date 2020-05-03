il launch file: lbr_iiwa_description gazebo_ctrl.launch
chiama il file world: lbr_iiwa.world
a cui è stata aggiunta l'istruzione:  <plugin name="gazebo_plugin_iiwa" filename="libgazebo_plugin_iiwa.so"/>

NOTA: controlla dopo aver chiamato catkin_make che il file libgazebo_plugin_iiwa.so sia presente nella cartella ros_ws/dev/plugin

