import math
import openrtdynamics2.lang as dy

from .path_data import *
from .tracking import *
from .vehicle_models import *
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
        Compute the noise-reduced path orientation Psi_r from curvature

        Ts       - the sampling time
        velocity - the driving velocity projected onto the path (d/dt d_star) 
        psi_rr   - noisy (e.g., due to sampling) path orientation
        K_r      - path curvature
        L        - gain for fusion using internal observer

        returns 
        
        psi_r_reconst     - the noise-reduced path orientation (reconstructed)
        psi_r_reconst_dot - the time derivative                (reconstructed)
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

def path_following(
        controller,
        par, 
        path, 
        x, y, psi, velocity, 
        Delta_l_r = 0.0, 
        Delta_l_r_dot = None, 
        psi_dot = None, 
        velocity_dot = None, 
        Delta_l_r_dotdot = None, 
        Ts=0.01
    ):
    """
        Basic steering control for path tracking and user-defined lateral error compensation
        
        Implements steering control for exact path following.
    
        Assumed is a kinematic bicycle model.
        Herein, the steering angle (delta) is the control variable. The variables
        x, y, psi, and velocity are measurements taken from the controlled system.
        The lateral offset Delta_l_r to the path is the reference for control.
        The optional signal Delta_l_r_dot describes the time derivative of Delta_l_r.
        
        controller - callback function that defines the error compensating controller
        par        - parameters that are passed to the callback
        Ts         - the sampling time

        Return values
        -------------

        results = {}
        results['x_r']                      # the current x-position of the closest point on the reference path
        results['y_r']                      # the current y-position of the closest point on the reference path
        results['v_star']                   # the current velocity of the closest point on the reference path
        results['d_star']                   # the current distance parameter of the closest point on the reference path
        results['psi_r']                    # the current path-tangent orientation angle in the closest point on the reference path
        results['psi_r_dot']                # the time derivative of psi_r
        results['Delta_l_r']                # the reference to the distance to the path
        results['Delta_l_r_dot']            # optional: the time derivative of Delta_l_r
        results['Delta_l']                  # the distance to the closest point on the reference path
        results['Delta_u']                  # small steering delta
        results['delta']                    # the requested steering angle / the control variable

        in case Delta_l_r_dot, psi_dot, velocity_dot, and Delta_l_r_dotdot are given
        the steering derivatives can be computed.

        results['Delta_u_dot']              # the derivative of Delta_u
        results['delta_dot']                # the derivative of delta_dot


    """
    index_head, _ = path_horizon_head_index(path)

    # structure for output signals
    results = dy.structure()

    # track the evolution of the closest point on the path to the vehicles position
    minimal_number_of_path_samples_to_start = 5 # depends on tracker(); should be at least 2
    with dy.sub_if( index_head > minimal_number_of_path_samples_to_start, subsystem_name='tracker' ) as system:

        tracking_results = tracker(path, x, y)

        system.set_outputs( tracking_results.to_list() )
    tracking_results.replace_signals( system.outputs )


    output_valid              = tracking_results['minimal_distance_reached']
    need_more_path_input_data = tracking_results['reached_the_end_of_currently_available_path_data']


    # in case the lookup was successful, run further operations on the path
    # to generate references and run the controller.
    with dy.sub_if( output_valid, prevent_output_computation=False, subsystem_name='controller') as system:

        # ps - path sample
        ps = track_projection_on_path(path, x, y, tracking_results = tracking_results)

        #
        # project the vehicle velocity onto the path yielding v_star 
        #
        # Used formula inside project_velocity_on_path:
        #   v_star = d d_star / dt = v * cos( Delta_u ) / ( 1 - Delta_l * K(d_star) ) 
        #

        Delta_u = dy.signal() # feedback from control
        v_star = project_velocity_on_path(velocity, Delta_u, ps['Delta_l'], ps['K_r'])


        #
        # compute an enhanced (less noise) signal for the path orientation psi_r by integrating the 
        # curvature profile and fusing the result with ps['psi_r'] to mitigate the integration drift.
        #

        psi_r, psi_r_dot = compute_path_orientation_from_curvature( Ts, v_star, ps['psi_r'], ps['K_r'], L=1.0 )


        #
        # controller callback
        #

        references = {
            'Delta_l_r'     : Delta_l_r,
            'Delta_l_r_dot' : Delta_l_r_dot,
            'Delta_l_r_dotdot' : Delta_l_r_dotdot
        }

        # Delta_l_dot might be further computed which improves the accuracy of the derivatives
        # in case of strong feedback control activity.  
        Delta_l_dot = None  # TODO: implement

        measurements = {
            'velocity'     : velocity,
            'velocity_dot' : velocity_dot,
            'psi'          : psi,
            'psi_dot'      : psi_dot,
            'Delta_l'      : ps['Delta_l'],
            'Delta_l_dot'  : Delta_l_dot
        }

        u, u_dot = controller( references, measurements, par )


        #
        # path tracking
        #
        # resulting lateral model u --> Delta_l : 1/s
        #

        Delta_u << dy.asin( dy.saturate(u / velocity, -0.99, 0.99) )
        delta = dy.unwrap_angle(angle=psi_r - psi + Delta_u, normalize_around_zero = True)

        # compute the derivatives of the steering angle (steering rate)
        if psi_dot is not None and Delta_l_r_dot is not None and velocity_dot is not None and Delta_l_r_dotdot is not None:

            Delta_u_dot = dy.cos( u / velocity ) * ( velocity * u_dot - velocity_dot * u ) / ( velocity*velocity )
            delta_dot = psi_r_dot - psi_dot + Delta_u_dot

            results['Delta_u_dot']       = Delta_u_dot
            results['delta_dot']         = delta_dot



        # collect resulting signals
        results['x_r']           = ps['x_r']      # the current x-position of the closest point on the reference path
        results['y_r']           = ps['y_r']      # the current y-position of the closest point on the reference path
        results['v_star']        = v_star         # the current velocity of the closest point on the reference path
        results['d_star']        = ps['d_star']   # the current distance parameter of the closest point on the reference path
        results['psi_r']         = psi_r          # the current path-tangent orientation angle in the closest point on the reference path
        results['K_r']           = ps['K_r']      # the curvature
        results['psi_r_dot']     = psi_r_dot      # the time derivative of psi_r

        results['Delta_u']       = Delta_u        # small steering delta
        results['delta']         = delta          # the requested steering angle / the control variable 

        results['Delta_l']       = ps['Delta_l']  # the distance to the closest point on the reference path
        results['Delta_l_dot']   = dy.float64(math.nan)  # d/dt Delta_l   TODO: implement


        # results['line_tracking_internals']  = ps['internals']


        # return
        system.set_outputs( results.to_list() )
    results.replace_signals( system.outputs )

    results['tracked_index'] = tracking_results['tracked_index']
    results['Delta_l_r']     = Delta_l_r      # the reference to the distance to the path

    results['need_more_path_input_data']     = need_more_path_input_data
    results['output_valid']                  = output_valid

    results['read_position']         = results['tracked_index'] + 1
    results['minimal_read_position'] = results['read_position'] - 100

    return results








def path_following_controller_P( 
        path, 
        x, y, psi, velocity, 
        Delta_l_r = 0.0, 
        Delta_l_r_dot = None, 
        k_p=2.0, Ts=0.01, 
        psi_dot = None, 
        velocity_dot = None, 
        Delta_l_r_dotdot = None, 
    ):
    """
        Basic steering control for path tracking using proportional lateral error compensation
            
        Implements steering control for exact path following combined with a P-controller to 
        control the lateral distance to the path.
    
        Assumed is a kinematic bicycle model.
        Herein, the steering angle (delta) is the control variable. The variables
        x, y, psi, and velocity are measurements taken from the controlled system.
        The lateral offset Delta_l_r to the path is the reference for control.
        The optional signal Delta_l_r_dot describes the time derivative of Delta_l_r.

        path  - the path/horizon to follow
        Ts    - the sampling time
        k_p   - proportional controller gain

        Return values - same as in path_following()
        -------------------------------------------

        results = {}
        results['x_r']                      # the current x-position of the closest point on the reference path
        results['y_r']                      # the current y-position of the closest point on the reference path
        results['v_star']                   # the current velocity of the closest point on the reference path
        results['d_star']                   # the current distance parameter of the closest point on the reference path
        results['psi_r']                    # the current path-tangent orientation angle in the closest point on the reference path
        results['psi_r_dot']                # the time derivative of psi_r
        results['Delta_l_r']                # the reference to the distance to the path
        results['Delta_l_r_dot']            # optional: the time derivative of Delta_l_r
        results['Delta_l']                  # the distance to the closest point on the reference path
        results['Delta_u']                  # small steering delta
        results['delta']                    # the requested steering angle / the control variable

        in case Delta_l_r_dot, psi_dot, velocity_dot, and Delta_l_r_dotdot are given
        the steering derivatives can be computed.

        results['Delta_u_dot']              # the derivative of Delta_u
        results['delta_dot']                # the derivative of delta_dot


    """

    def controller( references, measurements, par ):
        """
            P-controller to compensate for the lateral error to the path
        """

        Delta_l_dot      = measurements['Delta_l_dot']
        Delta_l_r_dot    = references['Delta_l_r_dot']
        Delta_l_r_dotdot = references['Delta_l_r_dotdot']

        # replace unavalable time derivatives with zero
        if Delta_l_dot is None:
            Delta_l_dot   = 0.0

        if Delta_l_r_dot is None:
            Delta_l_r_dot = 0.0

        if Delta_l_r_dotdot is None:
            Delta_l_r_dotdot = 0.0

        #
        # 2-DoF feedback control; herein, Delta_l_r_dot is the feed forward component
        #
        # The assumed model is G(s) = 1/s, which is the augmented system under active path following
        # and linearising control.
        #
        u = Delta_l_r_dot        + par['k_p'] * (references['Delta_l_r'] - measurements['Delta_l'])

        # time derivative of u: du/dt
        u_dot = Delta_l_r_dotdot + par['k_p'] * ( Delta_l_r_dot - Delta_l_dot )

        return u, u_dot



    par = {
        'k_p' : k_p
    }

    # path following and linearising control
    results = path_following(
        controller,            # callback to the implementation of P-control
        par,                   # parameters to the callback
        path, 
        x, y, psi, velocity, 
        Delta_l_r, 
        Delta_l_r_dot, 
        psi_dot, 
        velocity_dot, 
        Delta_l_r_dotdot, 
        Ts
    )

    return results





def path_lateral_modification2(
        input_path, 
        par,
        Ts,
        wheelbase, 
        velocity, 
        Delta_l_r, 
        Delta_l_r_dot, 
        Delta_l_r_dotdot, 
        d0, x0, y0, psi0, delta0, delta_dot0
    ):

    """
        Take an input path, modify it according to a given lateral distance profile,
        and generate a new path.

        Technically this combines a controller that causes an simulated vehicle to follows 
        the input path with defined lateral modifications. 

        Note: this implementation is meant as a callback routine for async_path_data_handler()
    """
    # create placeholders for the plant output signals
    x       = dy.signal()
    y       = dy.signal()
    psi     = dy.signal()
    psi_dot = dy.signal()

    if 'lateral_controller' not in par:
        # controller
        results = path_following_controller_P(
            input_path,

            x, y, psi,
            velocity,

            Delta_l_r        = Delta_l_r,
            Delta_l_r_dot    = Delta_l_r_dot,
            Delta_l_r_dotdot = Delta_l_r_dotdot,

            psi_dot          = dy.delay(psi_dot),
            velocity_dot     = dy.float64(0),

            Ts               = Ts,
            k_p              = 1
        )
    else:

        # path following and a user-defined linearising controller
        results = path_following(
            par['lateral_controller'],            # callback to the implementation of P-control
            par['lateral_controller_par'],        # parameters to the callback

            input_path,

            x, y, psi, 
            velocity, 

            Delta_l_r         = Delta_l_r, 
            Delta_l_r_dot     = Delta_l_r_dot, 
            Delta_l_r_dotdot  = Delta_l_r_dotdot, 

            psi_dot           = dy.delay(psi_dot), 
            velocity_dot      = dy.float64(0),  # velocity_dot 

            Ts                = Ts
        )


    #
    # The model of the vehicle
    #

    with dy.sub_if( results['output_valid'], prevent_output_computation=False, subsystem_name='simulation_model') as system:

        results['output_valid'].set_name('output_valid')

        results['delta'].set_name('delta')

        # steering angle limit
        limited_steering = dy.saturate(u=results['delta'], lower_limit=-math.pi/2.0, upper_limit=math.pi/2.0)

        # the model of the vehicle
        x_, y_, psi_, x_dot, y_dot, psi_dot_ = discrete_time_bicycle_model(
            limited_steering, 
            velocity, 
            Ts, wheelbase,
            x0, y0, psi0
            )

        # driven distance
        d = dy.euler_integrator(velocity, Ts, initial_state=d0)

        # outputs
        model_outputs = dy.structure(
            d       = d,
            x       = x_,
            y       = y_,
            psi     = psi_,
            psi_dot = dy.delay(psi_dot_) # NOTE: delay introduced to avoid algebraic loops, wait for improved ORTD
        )
        system.set_outputs(model_outputs.to_list())
    model_outputs.replace_signals( system.outputs )



    # close the feedback loops
    x       << model_outputs['x']
    y       << model_outputs['y']
    psi     << model_outputs['psi']
    psi_dot << model_outputs['psi_dot']

    #
    output_path = dy.structure({
        'd'       : model_outputs['d'],
        'x'       : model_outputs['x'],
        'y'       : model_outputs['y'],
        'psi'     : psi,
        'psi_dot' : psi_dot,

        'psi_r' : psi       + results['delta'],                   # orientation angle of the path the vehicle is drawing
        'K'     : ( psi_dot + results['delta_dot'] ) / velocity , # curvature of the path the vehicle is drawing

        'delta'         : results['delta'],
        'delta_dot'     : results['delta_dot'],

        'd_star'        : results['d_star'],
        'tracked_index' : results['tracked_index'],

        'output_valid'              : results['output_valid'],
        'need_more_path_input_data' : results['need_more_path_input_data'],

        'read_position'             : results['read_position'],
        'minimal_read_position'     : results['minimal_read_position']
    })

    return output_path

