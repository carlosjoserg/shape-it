# Informs that a cylinder model has been fit at a given location

# The pose gives you the transform that take you to the coordinate system
# of the cylinder, with the origin at the middle and the z axis aligned to
# the cylinder axis
geometry_msgs/PoseStamped pose



# These values give you the observed extents of the cylinder, the height
# along z, and the radius along x and y, in the cylinder's own coordinate 
# system (above)
float32 h
float32 r
