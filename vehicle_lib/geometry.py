import math
import openrtdynamics2.lang as dy


#
# geometry
#

def distance_between( x1, y1, x2, y2 ):
    """
        Compute the Euclidian distance between two points (x1,y1) and (x2,y2)
    """

    dx_ = x1 - x2
    dy_ = y1 - y2

    return dy.sqrt(  dx_*dx_ + dy_*dy_ )


def distance_to_line(x_s, y_s, x_e, y_e, x_test, y_test):
    """
        compute the shortest distance to a line

        returns a negative distance in case  (x_test, y_test) is to the left of the vector pointing from
        (x_s, y_s) to (x_e, y_e)
    """
    Delta_x = x_e - x_s
    Delta_y = y_e - y_s

    x_test_ = x_test - x_s
    y_test_ = y_test - y_s

    psi = dy.atan2(Delta_y, Delta_x)
    test_ang = dy.atan2(y_test_, x_test_)
    delta_angle = dy.unwrap_angle(test_ang - psi,  normalize_around_zero=True)

    length_s_to_test = dy.sqrt(x_test_ * x_test_ + y_test_ * y_test_)

    distance = dy.sin(delta_angle) * length_s_to_test
    distance_s_to_projection = dy.cos(delta_angle) * length_s_to_test

    return distance, distance_s_to_projection

