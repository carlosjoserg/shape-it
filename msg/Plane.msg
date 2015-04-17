# Informs that a plane has been detected at a given location

# The pose gives you the transform that take you to the coordinate system
# of the plane, with the origin somewhere in the table plane and the 
# z axis normal to the plane
geometry_msgs/PoseStamped pose



# These values give you the observed extents of the table, along x and y,
# in the table's own coordinate system (above)
# there is no guarantee that the origin of the table coordinate system is
# inside the boundary defined by these values. 
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_max
float64 z_min



# These values give you the coefficients of a plane
 
#float32 a
#float32 b
#float32 c
#float32 d