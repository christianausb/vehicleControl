import math
import openrtdynamics2.lang as dy



#
# safety
#

def compute_steering_constraints( v, v_dot, psi_dot, delta, a_l_min, a_l_max ):
    """
        Compute constraints for the steering angle and its rate so that the acceleration is bounded 

        delta  - the steering angle state of the vehicle (i.e., not the unsaturated control command)
    """

    delta_min = dy.float64(-1.0)
    delta_max = dy.float64(1.0)

    # note: check this proper min/max
    delta_dot_min = ( a_l_min - v_dot * dy.sin(delta) ) / ( v * dy.cos(delta) ) - psi_dot
    delta_dot_max = ( a_l_max - v_dot * dy.sin(delta) ) / ( v * dy.cos(delta) ) - psi_dot

    return delta_min, delta_max, delta_dot_min, delta_dot_max
