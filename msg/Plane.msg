# Represents a plane in point normal form. In this representation, we define a
# position (P, which is the reference frame of the header) and a non-zero
# vector (n). The end of the vector is a point on our plane. The rest of the
# plane is perpendicular from this vector.

# For more information, please consult:
# https://en.wikipedia.org/wiki/Plane_(geometry)#Point%E2%80%93normal_form_and_general_form_of_the_equation_of_a_plane

# However, given the reality of ROS transform trees, you sometimes want
# that vector to be zero. In this case, observe only the
# orientation produced by (n), set the boolean
# n_vector_is_zero to TRUE, and use the point in the
# header instead

Header header
geometry_msgs/Vector3 n
bool n_vector_is_zero
