# One unmodeled obstacle, as an axis-aligned box in the frame named by the enclosing
# ObstacleObservation header. Axis-aligned because a depth-derived cluster carries no reliable
# orientation, and an oriented box fitted to one would claim a precision the evidence does not
# have. The box is the tight bound of the voxels the cluster occupies, so it is conservative in
# every direction.

# Box centre, in the observation frame.
geometry_msgs/Point center

# Full extents along the observation frame axes. Every component is strictly positive.
geometry_msgs/Vector3 size

# Depth samples that survived cropping and self-filtering inside this cluster. Reported so a
# reader can tell a solid obstacle from a handful of stray returns without re-deriving it.
uint32 point_count
