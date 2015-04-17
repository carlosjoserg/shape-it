# Contains all the information from one run of the tabletop detection node

# The information for the plane that has been detected
Table table

# The information for the cylinders that has been detected from clusters, should match number of clusters
# Cylinder cylinder

# The raw clusters detected in the scan 
sensor_msgs/PointCloud[] clusters

# Whether the detection has succeeded or failed
int32 NO_CLOUD_RECEIVED = 1
int32 NO_TABLE = 2
int32 OTHER_ERROR = 3
int32 SUCCESS = 4
int32 result