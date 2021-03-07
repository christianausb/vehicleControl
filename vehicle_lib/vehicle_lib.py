import math
import numpy as np
import openrtdynamics2.lang as dy

"""
A library for modelling and control of vehicles than can be descirbed by the kinematic bicycle model.

The methods are based on the paper

Klauer C., Schwabe M., and Mobalegh H., "Path Tracking Control for Urban Autonomous Driving", IFAC World Congress 2020, Berlin

It implements
- A kinematic model to describe the position of the front axle given velocity and steering
- Acceleration at the front axle
- Dynamic tracking of the closest point of the vehicle front axle to the path using a computationally efficient tracing algorithm 
- Path following control in which the steering angle is adjusted such that the front axle follows a given path. 
  The approach is independent of the driving velocity, which must be measured though and passed to the controller.
  A velocity controller for the vehicle can be implemented separately.

The implementation is based on the framework OpenRTDynamics 2 for modelling dynamic systems (Simulink-like, but text-based).
Hence, efficient c++ code can be generated.
"""

# helper functions functions to design signals 
def co(time, val, Ts=1.0):
    return val * np.ones(int(math.ceil(time / Ts)))

def cosra(time, val1, val2, Ts=1.0):
    N = int(math.ceil(time / Ts))
    return val1 + (val2-val1) * (0.5 + 0.5 * np.sin(math.pi * np.linspace(0, 1, N) - math.pi/2))

def ra(time, val1, val2, Ts=1.0):
    N = int(math.ceil(time / Ts))
    return np.linspace(val1, val2, N)


#
# Path storage and access
#

def import_path_data(data):
    """
        Create a data structure containing the driving path 
    """
    # distance on path (D), position (X/Y), path orientation (PSI), curvature (K)
    path = {}
    path['D']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['D'] )
    path['X']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['X'] )
    path['Y']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['Y'] )
    path['PSI'] = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['PSI'] )
    path['K']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['K'] )

    path['samples'] = len( data['D'] )

    return path


def sample_path(path, index):
    """
        Read a sample of the given path at a given array-index
    """

    d   = dy.memory_read( memory=path['D'], index=index ) 
    y   = dy.memory_read( memory=path['Y'], index=index ) 
    x   = dy.memory_read( memory=path['X'], index=index ) 
    psi = dy.memory_read( memory=path['PSI'], index=index )
    K   = dy.memory_read( memory=path['K'], index=index )

    return d, x, y, psi, K


def sample_path_finite_difference(path, index):
    """
        Compute path orientation angle form x/y data only using finite differences
    """

    y1 = dy.memory_read( memory=path['Y'], index=index ) 
    y2 = dy.memory_read( memory=path['Y'], index=index + dy.int32(1) )

    x1 = dy.memory_read( memory=path['X'], index=index ) 
    x2 = dy.memory_read( memory=path['X'], index=index + dy.int32(1) )

    Delta_x = x2 - x1
    Delta_y = y2 - y1

    psi_r = dy.atan2(Delta_y, Delta_x)
    x_r = x1
    y_r = y1

    return x_r, y_r, psi_r



#
# vehicle models
#

def discrete_time_bicycle_model(delta, v, Ts, wheelbase, x0=0.0, y0=0.0, psi0=0.0):
    """
        Implement an ODE solver (Euler) for the kinematic bicycle model equations

        x, y           - describes the position of the front axle,
        delta          - the steering angle
        v              - the velocity measured on the front axle
        wheelbase      - the distance between front- and rear axle
        Ts             - the sampling time for the Euler integration
        psi            - the orientation of the carbody
        (x0, y0, psi0) - the initial states of the ODEs
    """

    x   = dy.signal()
    y   = dy.signal()
    psi = dy.signal()

    # bicycle model
    x_dot   = v * dy.cos( delta + psi )
    y_dot   = v * dy.sin( delta + psi )
    psi_dot = v / dy.float64(wheelbase) * dy.sin( delta )

    # integrators
    x    << dy.euler_integrator(x_dot,   Ts, x0)
    y    << dy.euler_integrator(y_dot,   Ts, y0)
    psi  << dy.euler_integrator(psi_dot, Ts, psi0)

    return x, y, psi, x_dot, y_dot, psi_dot

def compute_accelearation( v, v_dot, delta, delta_dot, psi_dot ):
    """
        Compute the acceleration at the front axle
    """

    a_lat = v_dot * dy.sin( delta ) + v * ( delta_dot + psi_dot ) * dy.cos( delta )
    a_long = None

    return a_lat, a_long


def lateral_vehicle_model(u_delta, v, v_dot, Ts, wheelbase, x0=0.0, y0=0.0, psi0=0.0, delta0=0.0, delta_disturbance=None, delta_factor=None):
    """
    Am implementation of the for bicycle model and the acceleration at the front axle
    """

    # add malicious factor to steering
    if delta_factor is not None:
        delta_ = u_delta * delta_factor
    else:
        delta_ = u_delta 

    # add disturbance to steering
    if delta_disturbance is not None:
        delta_ = delta_ + delta_disturbance

    # saturate steering
    delta = dy.saturate(u=delta_, lower_limit=-math.pi/2.0, upper_limit=math.pi/2.0)

    # bicycle model
    x, y, psi, x_dot, y_dot, psi_dot = discrete_time_bicycle_model(delta, v, Ts, wheelbase, x0, y0, psi0)

    #
    delta_dot = dy.diff( delta, initial_state=delta ) / dy.float64(Ts)
    delta = dy.delay( delta, delta0 )

    # compute acceleration in the point (x,y) in the vehicle frame
    a_lat, a_long = compute_accelearation( v, v_dot, delta, delta_dot, psi_dot )

    return x, y, psi, delta, delta_dot, a_lat, a_long, x_dot, y_dot, psi_dot


#
# line closest point tracker
#

def distance_between( x1, y1, x2, y2 ):
    """
        Compute the Euclidian distance between two points (x1,y1) and (x2,y2)
    """

    dx_ = x1 - x2
    dy_ = y1 - y2

    return dy.sqrt(  dx_*dx_ + dy_*dy_ )


def tracker(path, x, y):
    """
        Continuously project the point (x, y) onto the given path (closest distance)

        This is an internal function. C.f. track_projection_on_path for details and assumptions.

        returns
            index_track_next - the index in the path array for the closest distance to (x, y)
            Delta_index      - the change of the index to the previous lookup
            distance         - the absolute value of the closest distance of (x, y) to the path
    """
    index_track   = dy.signal()

    with dy.sub_loop( max_iterations=1000 ) as system:

        search_index_increment = dy.int32(1) # positive increment assuming positive velocity

        Delta_index = dy.sum(search_index_increment, initial_state=-1 )
        Delta_index_previous_step = Delta_index - search_index_increment

        x_test = dy.memory_read( memory=path['X'], index=index_track + Delta_index ) 
        y_test = dy.memory_read( memory=path['Y'], index=index_track + Delta_index )

        distance = distance_between( x_test, y_test, x, y )

        distance_previous_step = dy.delay(distance, initial_state=100000)
        minimal_distance_reached = distance_previous_step < distance

        # introduce signal names
        distance.set_name('tracker_distance')
        minimal_distance_reached.set_name('minimal_distance_reached')

        # break condition
        system.loop_until( minimal_distance_reached )

        # return        
        system.set_outputs([ Delta_index_previous_step, distance_previous_step ])


    Delta_index = system.outputs[0].set_name('tracker_Delta_index')
    distance = system.outputs[1].set_name('distance')

    index_track_next = index_track + Delta_index 

    index_track << dy.delay(index_track_next, initial_state=1)

    return index_track_next, Delta_index, distance


def track_projection_on_path(path, x, y):
    """
        Project the point (x, y) onto the given path (closest distance) yielding the parameter d_star.
        Return the properties of the path at d_star. Dynamic changes in (x, y) are continuously tracked.

        Assumption: special assumptions on the evolution of (x, y) are required:
        .) not leaving the a distance to the path of more than the curve radius at the closest distance.
        .) the projection-parameter d_star is assumed to increase over time. (I.e. in case (x,y) describes
           a vehicle, the velocity shall be positive and the direction of driving aligned to the 
           path +- 90 degrees)

        The implementation internally uses the function tracker() to perform an optimized tracking.

        returns

        d_star        - the optimal path parameter (distance along the path)
        x_r, y_r      - the coordinates of the path at d_star
        psi_rr, K_r   - the path orientation and curvature of the path at d_star
        Delta_l       - the clostest distance to the path (signed) 
        tracked_index - the index in the path array for the closest distance to (x, y)
        Delta_index   - the change of the index to the previous lookup
    """

    # track the evolution of the closest point on the path to the vehicles position
    tracked_index, Delta_index, closest_distance = tracker(path, x, y)

    # get the reference
    d_star, x_r, y_r, psi_rr, K_r = sample_path(path, index=tracked_index + dy.int32(1) )

    # add sign information to the distance
    Delta_l = distance_to_Delta_l( closest_distance, psi_rr, x_r, y_r, x, y )
  
    return d_star, x_r, y_r, psi_rr, K_r, Delta_l, tracked_index, Delta_index




def find_second_closest( path, x, y, index_star ):
    """
        Given the index of the clostest point, compute the index of the 2nd clostest point.
    """
        
    one = dy.int32(1)

    ip1 = index_star + one
    x_test_ip1 = dy.memory_read( memory=path['X'], index=ip1 ) 
    y_test_ip1 = dy.memory_read( memory=path['Y'], index=ip1 )
    distance_ip1 = distance_between( x, y, x_test_ip1, y_test_ip1 )

    im1 = index_star - one
    x_test_im1 = dy.memory_read( memory=path['X'], index=im1 ) 
    y_test_im1 = dy.memory_read( memory=path['Y'], index=im1 )
    distance_im1 = distance_between( x, y, x_test_im1, y_test_im1 )
    
    which = distance_ip1 > distance_im1

    second_clostest_distance = distance_ip1
    second_clostest_distance = dy.conditional_overwrite( second_clostest_distance, condition=which, new_value=distance_im1 )

    index_second_star = ip1
    index_second_star = dy.conditional_overwrite( index_second_star, condition=which, new_value=im1 )

    return second_clostest_distance, index_second_star



def compute_distance_from_linear_interpolation( d1, d2 ):

    # TODO: implement

    zero = dy.float64(0)

    return zero



def tracker_distance_ahead(path, current_index, distance_ahead):
    """
        Track a point on the path that is ahead to the closest point by a given distance


                  <----- Delta_index_track ----->
        array: X  X  X  X  X  X  X  X  X  X  X  X  X  X  X 
                  ^               
            current_index
    """

    if 'Delta_d' in path:
        # constant sampling interval in distance
        # computation can be simplified
        pass

    target_distance = dy.float64(distance_ahead) + dy.memory_read( memory=path['D'], index=current_index )


    def J( index ):

        d_test = dy.memory_read( memory=path['D'], index=index ) 
        distance = dy.abs( d_test - target_distance )

        return distance
    

    Delta_index_track   = dy.signal()

    # initialize J_star
    J_star_0 = J(current_index + Delta_index_track)
    J_star_0.set_name('J_star_0')

    #
    # compute the direction in which J has its decent
    # if true: with increasing index J increases  --> decrease search index
    # if false: with increasing index J decreases --> increase search index
    #
    J_next_index = J(current_index + Delta_index_track + dy.int32(1))
    J_Delta_to_next_index = J_next_index - J_star_0

    direction_flag = J_Delta_to_next_index > dy.float64(0)

    search_index_increment = dy.int32(1) 
    search_index_increment = dy.conditional_overwrite(search_index_increment, direction_flag, dy.int32(-1) )
    search_index_increment.set_name('search_index_increment')

    # loop to find the minimum of J
    with dy.sub_loop( max_iterations=1000 ) as system:


        # J_star(k) - the smallest J found so far
        J_star = dy.signal()  #.set_datatype( dy.DataTypeFloat64(1) )
        
        # inc- / decrease the search index
        #Delta_index = dy.sum(search_index_increment, initial_state=0, no_delay=True )

        Delta_index_prev_it, Delta_index = dy.sum2(search_index_increment, initial_state=0 )

        Delta_index.set_name('Delta_index')

        # sample the cost function and check if it got smaller in this step
        J_to_verify = J( current_index + Delta_index_track + Delta_index )
        J_to_verify.set_name('J_to_verify')

        step_caused_improvement = J_to_verify < J_star

        # replace the 
        J_star_next = dy.conditional_overwrite( J_star, step_caused_improvement, J_to_verify )

        # state for J_star
        J_star << dy.delay( J_star_next, initial_state=J_star_0 ).set_name('J_star')

        # loop break condition
        system.loop_until( dy.logic_not( step_caused_improvement ) )
        #system.loop_until( dy.int32(1) == dy.int32(0) )

        # return the results computed in the loop        
        system.set_outputs([ Delta_index_prev_it, J_to_verify, J_star ])

    Delta_index = system.outputs[0]

    Delta_index_track_next = Delta_index_track + Delta_index
    Delta_index_track << dy.delay(Delta_index_track_next, initial_state=0)
    Delta_index_track.set_name('Delta_index_track')

    # compute the residual distance
    optimal_distance = dy.memory_read( memory=path['D'], index=current_index + Delta_index_track_next )
    distance_residual = target_distance - optimal_distance

    return Delta_index_track_next, distance_residual, Delta_index 









def distance_to_Delta_l( distance, psi_r, x_r, y_r, x, y ):
    """
    Add sign information to a closest distance measurement 
    """
    psi_tmp = dy.atan2(y - y_r, x - x_r)
    delta_angle = dy.unwrap_angle( psi_r - psi_tmp, normalize_around_zero=True )
    sign = dy.conditional_overwrite(dy.float64(1.0), delta_angle > dy.float64(0) ,  -1.0  )
    Delta_l = distance * sign

    return Delta_l





#
# models assuming path following
#

def project_velocity_on_path(velocity, Delta_u, Delta_l, K_r):
    """
        Compute the velocity of the closest point on the path
    """

    #
    # This evaluates the formula
    #
    # v_star = d d_star / dt = v * cos( Delta_u ) / ( 1 - Delta_l * K(d_star) ) 
    #

    v_star = velocity * dy.cos( Delta_u ) / ( dy.float64(1) - Delta_l * K_r ) 

    return v_star

def compute_nominal_steering_from_curvature( Ts : float, l_r : float, v, K_r ):
    """
        compute the nominal steering angle and rate from given path heading and curvature
        signals.
    """

    psi_dot = dy.signal()

    delta_dot = v * K_r - psi_dot
    delta = dy.euler_integrator( delta_dot, Ts )
    psi_dot << (v / dy.float64(l_r) * dy.sin(delta))

    return delta, delta_dot, psi_dot

def compute_path_orientation_from_curvature( Ts : float, velocity, psi_rr, K_r, L ):
    """
        compute the noise-reduced path orientation Psi_r from curvature

        Ts       - the sampling time
        velocity - the driving velocity 
        psi_rr   - noisy (e.g., due to sampling) path orientation
        K_r      - path curvature
        L        - gain for fusion using internal observer

        returns 
        
        psi_r_reconst     - the noise-reduced path orientation
        psi_r_reconst_dot - the time derivative
    """

    eps = dy.signal()

    psi_r_reconst_dot = velocity * (K_r + eps)
    psi_r_reconst = dy.euler_integrator( u=psi_r_reconst_dot, Ts=Ts, initial_state=psi_rr )

    # observer to compensate for integration error
    eps << (psi_rr - psi_r_reconst) * dy.float64(L)

    return psi_r_reconst, psi_r_reconst_dot

def compute_nominal_steering_from_path_heading( Ts : float, l_r : float, v, psi_r ):
    """
        Compute the steering angle to follow a path given the path tangent angle

        Internally uses a (partial) model of the bicycle-vehicle to comput the
        optimal steering angle given the path orientation-angle. Internally,
        the orientation of the vehicle body (psi) is computed to determine the optimal
        steering angle.
    """

    psi = dy.signal()

    delta = psi_r - psi 
    psi_dot = v / dy.float64(l_r) * dy.sin(delta)

    psi << dy.euler_integrator( psi_dot, Ts )

    return delta, psi, psi_dot

#
# Path following control
#

def path_following_controller_basic( path, x, y, psi, velocity, Delta_l_r = 0.0, k_p=2.0, Ts=0.01  ):
    """
        Basic steering control for path tracking
        
        Path following steering control for exact path following and P-control to control the lateral 
        distance to the path are combined.
    
        Controlls a kinematic bicycle model (assumption) to follow the given path.
        Herein, the steering angle delta is the control variable. The variables
        x, y, psi, and velocity are measurements taken from the controlled system.
        The lateral offset Delta_l_r to the path is the reference.
        

        Return values
        -------------

        results = {}
        results['x_r']       = x_r          # the current x-position of the closest point on the reference path
        results['y_r']       = y_r          # the current y-position of the closest point on the reference path
        results['v_star']    = v_star       # the current velocity of the closest point on the reference path
        results['d_star']    = d_star       # the current distance parameter of the closest point on the reference path
        results['psi_r']     = psi_r        # the current path-tangent orientation angle in the closest point on the reference path
        results['psi_r_dot'] = psi_r_dot    # the time derivative of psi_r
        results['Delta_l_r'] = Delta_l_r    # the reference to the distance to the path
        results['Delta_l']   = Delta_l      # the distance to the closest point on the reference path
        results['Delta_u']   = Delta_u      # small steering delta
        results['delta']     = delta        # the requested steering angle / the control variable 

    """


    # track the evolution of the closest point on the path to the vehicles position
    d_star, x_r, y_r, psi_rr, K_r, Delta_l, tracked_index, Delta_index = track_projection_on_path(path, x, y)

    #
    # project the vehicle velocity onto the path yielding v_star 
    #
    # Used formula inside project_velocity_on_path:
    #   v_star = d d_star / dt = v * cos( Delta_u ) / ( 1 - Delta_l * K(d_star) ) 
    #

    Delta_u = dy.signal() # feedback from control
    v_star = project_velocity_on_path(velocity, Delta_u, Delta_l, K_r)


    #
    # compute an enhanced (less noise) signal for the path orientation psi_r by integrating the 
    # curvature profile and fusing the result with psi_rr to mitigate the integration drift.
    #

    psi_r, psi_r_dot = compute_path_orientation_from_curvature( Ts, v_star, psi_rr, K_r, L=1.0 )
    
    # feedback control
    u = dy.PID_controller(r=Delta_l_r, y=Delta_l, Ts=Ts, kp=k_p)

    # path tracking
    # resulting lateral model u --> Delta_l : 1/s
    Delta_u << dy.asin( dy.saturate(u / velocity, -0.99, 0.99) )
    delta = dy.unwrap_angle(angle=psi_r - psi + Delta_u, normalize_around_zero = True)

    
    # collect resulting signals
    results = {}
    results['x_r']       = x_r          # the current x-position of the closest point on the reference path
    results['y_r']       = y_r          # the current y-position of the closest point on the reference path
    results['v_star']    = v_star       # the current velocity of the closest point on the reference path
    results['d_star']    = d_star       # the current distance parameter of the closest point on the reference path
    results['psi_r']     = psi_r        # the current path-tangent orientation angle in the closest point on the reference path
    results['psi_r_dot'] = psi_r_dot    # the time derivative of psi_r
    results['Delta_l_r'] = Delta_l_r    # the reference to the distance to the path
    results['Delta_l']   = Delta_l      # the distance to the closest point on the reference path
    results['Delta_u']   = Delta_u      # small steering delta
    results['delta']     = delta        # the requested steering angle / the control variable 

    return results



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




#
# For unit-test / reference purposes
#

def lookup_closest_point( N, path_distance_storage, path_x_storage, path_y_storage, x, y ):
    """
        brute force implementation for finding a clostest point
    """
    #
    source_code = """
        // int N = 360;
        int N = *(&path_distance_storage + 1) - path_distance_storage;

        int i = 0;
        double min_distance_to_path = 1000000;
        int min_index = 0;

        for (i = 0; i < N; ++i ) {
            double dx = path_x_storage[i] - x_;
            double dy = path_y_storage[i] - y_;
            double distance_to_path = sqrt( dx * dx + dy * dy );

            if ( distance_to_path < min_distance_to_path ) {
                min_distance_to_path = distance_to_path;
                min_index = i;
            }
        }

        double dx_p1, dy_p1, dx_p2, dy_p2, distance_to_path_p1, distance_to_path_p2;

        dx_p1 = path_x_storage[min_index + 1] - x_;
        dy_p1 = path_y_storage[min_index + 1] - y_;
        distance_to_path_p1 = sqrt( dx_p1 * dx_p1 + dy_p1 * dy_p1 );

        dx_p2 = path_x_storage[min_index - 1] - x_;
        dy_p2 = path_y_storage[min_index - 1] - y_;
        distance_to_path_p2 = sqrt( dx_p2 * dx_p2 + dy_p2 * dy_p2 );

        int interval_start, interval_stop;
        if (distance_to_path_p1 < distance_to_path_p2) {
            // minimal distance in interval [min_index, min_index + 1]
            interval_start = min_index;
            interval_stop  = min_index + 1;
        } else {
            // minimal distance in interval [min_index - 1, min_index]
            interval_start = min_index - 1;
            interval_stop  = min_index;
        }

        // linear interpolation (unused)
        double dx = path_x_storage[interval_stop] - path_x_storage[interval_start] ;
        double dy = path_y_storage[interval_stop] - path_y_storage[interval_start] ;



        index_start   = interval_start;
        index_closest = min_index;
        distance      = min_distance_to_path;

    """
    array_type = dy.DataTypeArray( N, datatype=dy.DataTypeFloat64(1) )
    outputs = dy.generic_cpp_static(input_signals=[ path_distance_storage, path_x_storage, path_y_storage, x, y ], 
                                    input_names=[ 'path_distance_storage', 'path_x_storage', 'path_y_storage', 'x_', 'y_' ], 
                                    input_types=[ array_type, array_type, array_type, dy.DataTypeFloat64(1), dy.DataTypeFloat64(1) ], 
                                    output_names=[ 'index_closest', 'index_start', 'distance'],
                                    output_types=[ dy.DataTypeInt32(1), dy.DataTypeInt32(1), dy.DataTypeFloat64(1) ],
                                    cpp_source_code = source_code )

    index_start = outputs[0]
    index_closest = outputs[1]
    distance     = outputs[2]

    return index_closest, distance, index_start



def global_lookup_distance_index( path_distance_storage, path_x_storage, path_y_storage, distance ):
    #
    source_code = """
        index = 0;
        int i = 0;

        for (i = 0; i < 100; ++i ) {
            if ( path_distance_storage[i] < distance && path_distance_storage[i+1] > distance ) {
                index = i;
                break;
            }
        }
    """
    array_type = dy.DataTypeArray( 360, datatype=dy.DataTypeFloat64(1) )
    outputs = dy.generic_cpp_static(input_signals=[ path_distance_storage, path_x_storage, path_y_storage, distance ], 
                                    input_names=[ 'path_distance_storage', 'path_x_storage', 'path_y_storage', 'distance' ], 
                                    input_types=[ array_type, array_type, array_type, dy.DataTypeFloat64(1) ], 
                                    output_names=['index'],
                                    output_types=[ dy.DataTypeInt32(1) ],
                                    cpp_source_code = source_code )

    index = outputs[0]

    return index
