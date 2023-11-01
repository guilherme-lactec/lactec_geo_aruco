how to init and run the system 

1- this software should be able to track the position of object with aruco markers in it 
2 - to run the debug software, with rviz for vizualization ,  open an new terminal and type : geo_tracker_debug
 This alias will : 
    -> run the roslaunch file in /home/geo/ros_ws/src/lactec_geo/launch/launch/lactec_geo.launch
       -> this will init roscore
       -> init the canera node 
       -> init the aruco single node
       -> make the proper transformation 
       -> run the node that reads the transformation betwen the aruco and the map and pub its pose
       -> run the node that read the pose and save as csv 
       -> open the proper rviz file to view the camera with markers and the TF 



NOTE: this will run all the software in the same terminal 
if you want to debug a especific part coment the file in the launch file and run it in another terminal with rosrun 