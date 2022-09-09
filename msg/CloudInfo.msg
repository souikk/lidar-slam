# Cloud Info
std_msgs/Header header  

# gnss pose 
float32 gnss_lat
float32 gnss_lon
float32 gnss_h
float32 gnss_orientation_w
float32 gnss_orientation_x
float32 gnss_orientation_y
float32 gnss_orientation_z

# laser observe pose
float32 laser_position_x
float32 laser_position_y
float32 laser_position_z
float32 laser_orientation_w
float32 laser_orientation_x
float32 laser_orientation_y
float32 laser_orientation_z

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed

