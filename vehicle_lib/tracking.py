import math
import openrtdynamics2.lang as dy

from .path_data import *
from .geometry import *

#
# line closest point tracker
#


def continuous_optimization_along_path(path, current_index, J, par):
    """
        Minimize the given cost function by varying the index of the path array  


                  <----- Delta_index_track ----->
        array: X  X  X  X  X  X  X  X  X  X  X  X  X  X  X 
                  ^               
            current_index
    """


    if 'Delta_d' in path:
        # constant sampling interval in distance
        # computation can be simplified
        pass

    # get the highest available array index in the horizon
    index_head, _ = path_horizon_head_index(path)

    #
    # 
    #

    Delta_index_track = dy.signal()

    # initialize J_star
    J_star_0 = J(path, current_index + Delta_index_track, par)

    #
    # compute the direction (gradient) in which J has its decent
    # if true: with increasing index J increases  --> decrease search index
    # if false: with increasing index J decreases --> increase search index
    #
    J_prev_index = J( path, current_index + Delta_index_track - 1, par )
    J_Delta_to_next_index = J_star_0 - J_prev_index

    search_index_increment = dy.conditional_overwrite(dy.int32(1), J_Delta_to_next_index > 0, dy.int32(-1) )

    # loop to find the minimum of J
    with dy.sub_loop( max_iterations=1000, subsystem_name='optim_loop' ) as system:

        # J_star(k) - the smallest J found so far
        J_star = dy.signal()
        
        # inc- / decrease the search index
        Delta_index_previous_step, Delta_index = dy.sum2(search_index_increment, initial_state=0 )
        index_to_investigate = current_index + Delta_index_track + Delta_index

        # sample the cost function and check if it got smaller in this step
        J_to_verify = J( path, index_to_investigate, par )
        step_caused_improvement = J_to_verify < J_star

        # in case the step yielded a lower cost, replace the prev. minimal cost
        J_star_next = dy.conditional_overwrite( J_star, step_caused_improvement, J_to_verify )

        # state for J_star
        J_star << dy.delay( J_star_next, initial_state=J_star_0 )

        #
        # loop break conditions
        #

        # when reaching the end of the available data, stop the loop and indicate the need for extending the horizon
        reached_the_end_of_currently_available_path_data = index_to_investigate >= index_head # reached the end of the input data?

        # similarly check for the begin ...
        reached_the_begin_of_currently_available_path_data = index_to_investigate - 1 <= path_horizon_tail_index(path)[0]

        # in case the iteration did not reduce the cost, assume that the minimum was reached in the prev. iteration
        reached_minimum = dy.logic_not( step_caused_improvement ) 

        system.loop_until( 
            dy.logic_or(
                dy.logic_or( 
                    
                    reached_minimum, 
                    reached_the_end_of_currently_available_path_data 

                ),
                reached_the_begin_of_currently_available_path_data

            ).set_name('loop_until')
        )

        # assign signals names to appear in the generated source code
        J_star_0.set_name('J_star_0')
        search_index_increment.set_name('search_index_increment')
        J_star.set_name('J_star')
        Delta_index.set_name('Delta_index')
        index_head.set_name('index_head')
        index_to_investigate.set_name('index_to_investigate')
        J_to_verify.set_name('J_to_verify')
        step_caused_improvement.set_name('step_caused_improvement')

        # return  
        outputs = dy.structure()
        outputs['Delta_index']                                       = Delta_index_previous_step
        outputs['J_star']                                            = J_star_next
        outputs['reached_minimum']                                   = reached_minimum
        outputs['reached_the_end_of_currently_available_path_data']  = reached_the_end_of_currently_available_path_data
        outputs['index_head']                                        = index_head * 1
        outputs['index_to_investigate']                              = index_to_investigate
        outputs['J_to_verify']                                       = J_to_verify

        system.set_outputs(outputs.to_list())
    outputs.replace_signals( system.outputs )


    Delta_index                                      = outputs['Delta_index'] 
    J_star                                           = outputs['J_star'] 
    reached_minimum                                  = outputs['reached_minimum'] 
    reached_the_end_of_currently_available_path_data = outputs['reached_the_end_of_currently_available_path_data'] 

    # Introduce dy.sink(signal) in ORTD to ensure the given signals is not optimized out and becomes visible in the debugging traces
    dummy = 0 * outputs['index_head'] + 0 * outputs['index_to_investigate'] + 0 * outputs['J_to_verify']


    Delta_index_track_next = Delta_index_track + Delta_index
    Delta_index_track << dy.delay(Delta_index_track_next, initial_state=1) # start at 1 so that the backwards gradient can be computed at index=1
    Delta_index_track.set_name('Delta_index_track')


    # optimal index
    optimal_index = current_index + Delta_index_track_next


    results = dy.structure()
    results['optimal_index']                                    = optimal_index
    results['J_star']                                           = J_star           + 0 * dummy
    results['Delta_index']                                      = Delta_index
    results['Delta_index_track_next']                           = Delta_index_track_next
    results['reached_minimum']                                  = reached_minimum
    results['reached_the_end_of_currently_available_path_data'] = reached_the_end_of_currently_available_path_data

    return results





#
# line closest point tracker
#


def tracker(path, x, y):
    """
        Continuously project the point (x, y) onto the given path (closest distance)

        This is an internal function. C.f. track_projection_on_path for details and assumptions.

        returns in structure tracking_results:
            tracked_index    - the index in the path array for the closest distance to (x, y)
            Delta_index      - the change of the index to the previous lookup
            distance         - the absolute value of the closest distance of (x, y) to the path

            reached_the_end_of_currently_available_path_data
                             - reached the end of the path
    """

    #
    # define the optimization problem
    #

    # pack parameters
    par = ( x, y )

    def J( path, index, par ):

        # unpack parameters
        x, y = par

        # cost function J: index -> distance
        x_test, y_test        = sample_path_xy( path, index )
        distance              = distance_between( x_test, y_test, x, y )

        # cost
        return distance
    


    #
    # continuous optimization
    #

    results = continuous_optimization_along_path(
        path, 
        dy.int32(0), 
        J, 
        par
    )

    #
    distance = results['J_star']
    minimal_distance_reached = results['reached_minimum']

    #
    tracking_results = dy.structure()
    tracking_results['tracked_index']                                    = results['optimal_index']
    tracking_results['Delta_index']                                      = results['Delta_index']
    tracking_results['distance']                                         = distance
    tracking_results['minimal_distance_reached']                         = minimal_distance_reached
    tracking_results['reached_the_end_of_currently_available_path_data'] = results['reached_the_end_of_currently_available_path_data']

    return tracking_results



def tracker_distance_ahead(path, current_index, distance_ahead):
    """
        Track a point on the path that is ahead to the closest point by a given distance


                  <----- Delta_index_track ----->
        array: X  X  X  X  X  X  X  X  X  X  X  X  X  X  X 
                  ^               
            current_index
    """

    #
    # define the optimization problem
    #
    target_distance = dy.float64(distance_ahead) + sample_path_d(path, current_index)

    # pack parameters
    par = ( target_distance, )

    def J( path, index, par ):

        # unpack parameters
        target_distance = par[0]

        # cost
        d = sample_path_d(path, index)
        cost   = dy.abs( d - target_distance )

        return cost
    


    #
    # continuous optimization
    #

    results = continuous_optimization_along_path(
        path, 
        current_index, 
        J, 
        par
    )

    # compute the residual distance    
    optimal_distance = sample_path_d(path, index=results['optimal_index'])
    distance_residual = target_distance - optimal_distance

    return results['Delta_index_track_next'], distance_residual, results['Delta_index'] 






def _get_line_segment( path, x, y, index_star ):
    """
        Given the index of the clostest point, compute the index of the 2nd clostest point.
    """
        
    one = dy.int32(1)

    x_star, y_star = sample_path_xy(path, index=index_star)

    x_test_ip1, y_test_ip1 = sample_path_xy(path, index=index_star + one)
    distance_ip1 = distance_between( x, y, x_test_ip1, y_test_ip1 )

    x_test_im1, y_test_im1 = sample_path_xy(path, index=index_star - one)
    distance_im1 = distance_between( x, y, x_test_im1, y_test_im1 )
    
    # find out which point is the 2nd closest
    # which = True  means that the point referred by the index index_star - 1 is the 2nd closest 
    # which = False means that the point referred by the index index_star + 1 is the 2nd closest 
    which = distance_ip1 > distance_im1

    second_clostest_distance = dy.conditional_overwrite( distance_ip1, condition=which, new_value=distance_im1 )

    index_second_star = dy.conditional_overwrite( index_star + one, condition=which, new_value=index_star - one )

    #
    i_s = dy.conditional_overwrite( index_star,        condition=which, new_value=index_star - one )
    i_e = dy.conditional_overwrite( index_star + one , condition=which, new_value=index_star )

    #
    # get start/end xy-points of the line segment which is closest
    # the line is described by (x_s, y_s) --> (x_e, y_e)
    #

    # find start point (x_s, y_s)
    x_s = dy.conditional_overwrite( x_star, condition=which, new_value=x_test_im1 )
    y_s = dy.conditional_overwrite( y_star, condition=which, new_value=y_test_im1 )

    # find stop point (x_e, y_e)
    x_e = dy.conditional_overwrite( x_test_ip1, condition=which, new_value=x_star )
    y_e = dy.conditional_overwrite( y_test_ip1, condition=which, new_value=y_star )

    return i_s, i_e, x_s, y_s, x_e, y_e, index_second_star, second_clostest_distance





def _distance_to_Delta_l( distance, psi_r, x_r, y_r, x, y ):
    """
        Add sign information to a closest distance measurement 
    """
    psi_tmp = dy.atan2(y - y_r, x - x_r)
    delta_angle = dy.unwrap_angle( psi_r - psi_tmp, normalize_around_zero=True )
    sign = dy.conditional_overwrite(dy.float64(1.0), delta_angle > dy.float64(0) ,  -1.0  )
    Delta_l = distance * sign

    return Delta_l


def track_projection_on_path(path, x, y, tracking_results=None, use_linear_interpolation_in_sampling=True):
    """
        Project the point (x, y) onto the given path (closest distance) yielding the parameter d_star.
        Return the properties of the path at d_star. Dynamic changes in (x, y) are continuously tracked.

        Assumption: special assumptions on the evolution of (x, y) are required:
        .) not leaving the a distance to the path of more than the curve radius at the closest distance.
        .) the projection-parameter d_star is assumed to increase over time. (I.e., in case (x,y) describes
           a vehicle, the velocity shall be positive and the direction of driving aligned to the 
           path +- 90 degrees)

        The implementation internally uses the function tracker() to perform an optimized tracking.

        Returns
        -------

        d_star        - the optimal path parameter (distance along the path)
        x_r, y_r      - the coordinates of the path at d_star
        psi_r         - the path orientation angle 
        K_r           - the curvature of the path at d_star
        Delta_l       - the clostest distance to the path (signed) 
        tracked_index - the index in the path array for the closest distance to (x, y)
        Delta_index   - the change of the index to the previous lookup

        internals     - an (hash) array of some internals signals
    """

    # track the evolution of the closest point on the path to the vehicles position
    # tracked_index, Delta_index, closest_distance, minimal_distance_reached, reached_the_end_of_currently_available_path_data = tracker(path, x, y)

    # tr - tracking_results
    if tracking_results is not None:
        # using external tracker
        tr = tracking_results

    else:
        # using internal tracker
        tr = tracker(path, x, y)

    # The index 'tracked_index' is the index referring to the closest point to the path.
    # Now, find the index of the 2nd closest point

    i_s, i_e, x_s, y_s, x_e, y_e, index_2nd_closest, _ = _get_line_segment( path, x, y, tr['tracked_index'] )

    if use_linear_interpolation_in_sampling:
        # use linear interpolation of the line between the path xy-samples
        # the line is described by the start/end points (x_s, y_s) ---> (x_e, y_e)

        Delta_l_lin_interpol, distance_s_to_projection = distance_to_line(x_s, y_s, x_e, y_e, x_test=x, y_test=y)
        interpolation_factor = distance_s_to_projection / distance_between( x_s, y_s, x_e, y_e )


    #
    # sample the path
    #

    if use_linear_interpolation_in_sampling:
        # zero-order hold style sampling
        d_star, x_r, y_r, psi_r, K_r = sample_path_linear_interpolation(path, i_s, i_e, interpolation_factor)
    else:
        # linear interpolation
        d_star, x_r, y_r, psi_r, K_r = sample_path(path, index=i_s )

    #
    # compute the distance to closest sample
    #

    Delta_l_closest_path_sample = _distance_to_Delta_l( tr['distance'], psi_r, x_r, y_r, x, y )


    # Finally assign the best estimate of the lateral distance to the path
    if use_linear_interpolation_in_sampling:
        Delta_l = Delta_l_lin_interpol
    else:
        Delta_l = Delta_l_closest_path_sample


    #
    # return internal signals
    #

    internals={}

    if use_linear_interpolation_in_sampling:
        internals['Delta_l_lin_interpol'] = Delta_l_lin_interpol

        internals['interpolation_factor'] = interpolation_factor

    internals['i_s'] = i_s
    internals['i_e'] = i_e
    internals['x_s'] = x_s
    internals['y_s'] = y_s
    internals['x_e'] = x_e
    internals['y_e'] = y_e

    internals['Delta_l_closest_path_sample'] = Delta_l_closest_path_sample

    internals['i_star_2']    = index_2nd_closest
    #internals['i_star']      = tr['tracked_index']
    #internals['Delta_index'] = tr['Delta_index'] 

    #
    sample = {}
    sample['d_star']  = d_star
    sample['x_r']     = x_r
    sample['y_r']     = y_r
    sample['psi_r']   = psi_r
    sample['K_r']     = K_r
    sample['Delta_l'] = Delta_l

    sample['tracked_index'] = tr['tracked_index']
    sample['Delta_index']   = tr['Delta_index']

    sample['internals']   = internals

    return sample


