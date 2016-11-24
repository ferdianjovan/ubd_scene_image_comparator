std_msgs/Header header
sensor_msgs/Image image         # RGB cutouts of the UBDs, 3x height.
geometry_msgs/Point[] ubd_pos   # The UBDs in world-coordinates.
geometry_msgs/Point[] cd_pos    # only use x and y 
bool annotated
