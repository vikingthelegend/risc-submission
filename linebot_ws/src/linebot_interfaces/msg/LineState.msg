# Output of the line detector, consumed by the follower / PID node.
# Lateral errors are normalised to [-1, 1]:  0 = line at image centre,
# +1 = line at right image edge, -1 = line at left image edge.
std_msgs/Header header

bool    detected          # true if at least one slice found the line
float32 error             # weighted error across detected slices (feed this to PID)
float32 near_error        # error of the nearest detected slice
float32 far_error         # error of the farthest detected slice (look-ahead)
uint8   slices_detected   # how many slices saw the line
