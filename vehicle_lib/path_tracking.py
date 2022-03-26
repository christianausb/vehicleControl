import openrtdynamics2.lang as dy
import openrtdynamics2.py_execute as dyexe
import openrtdynamics2.targets as tg

from .path_data import *
from .tracking import *
from .path_following import project_velocity_on_path, compute_path_orientation_from_curvature







def path_tracking(
        path, 
        par,
        Ts,
        velocity, velocity_dot,
        x, y, x_dot, y_dot,
        Delta_u, Delta_u_dot
    ):

    """
        Take an input path, modify it according to a given lateral distance profile,
        and generate a new path.

        Technically this combines a controller that causes an simulated vehicle to follows 
        the input path with defined lateral modifications. 

        Note: this implementation is meant as a callback routine for async_path_data_handler()
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

        v_star = project_velocity_on_path(velocity, Delta_u, ps['Delta_l'], ps['K_r'])


        #
        # compute an enhanced (less noise) signal for the path orientation psi_r by integrating the 
        # curvature profile and fusing the result with ps['psi_r'] to mitigate the integration drift.
        #

        psi_r, psi_r_dot = compute_path_orientation_from_curvature( Ts, v_star, ps['psi_r'], ps['K_r'], L=1.0 )



        # collect resulting signals
        results['x_r']           = ps['x_r']      # the current x-position of the closest point on the reference path
        results['y_r']           = ps['y_r']      # the current y-position of the closest point on the reference path
        results['v_star']        = v_star         # the current velocity of the closest point on the reference path
        results['d_star']        = ps['d_star']   # the current distance parameter of the closest point on the reference path
        results['psi_r']         = psi_r          # the current path-tangent orientation angle in the closest point on the reference path
        results['psi_r_dot']     = psi_r_dot      # the time derivative of psi_r
        results['K_r']           = ps['K_r']      # the curvature

        results['Delta_l']       = ps['Delta_l']  # the distance to the closest point on the reference path
        results['Delta_l_dot']   = dy.float64(0.0)  # d/dt Delta_l   TODO: implement



        # results['line_tracking_internals']  = ps['internals']


        # return
        system.set_outputs( results.to_list() )
    results.replace_signals( system.outputs )




    #
    output_path = dy.structure({

        'tracked_index' : tracking_results['tracked_index'],
        'd_star'        : results['d_star'],
        'v_star'        : results['v_star'],
        'x_r'           : results['x_r'],
        'y_r'           : results['y_r'],
        'psi_r'         : results['psi_r'],
        'psi_r_dot'     : results['psi_r_dot'],

        'K_r'           : results['K_r'],

        'Delta_l'       : results['Delta_l'],
        'Delta_l_dot'   : results['Delta_l_dot'],

        'output_valid'              : output_valid,
        'need_more_path_input_data' : need_more_path_input_data,

        'read_position'         : tracking_results['tracked_index'] + 1,
        'minimal_read_position' : tracking_results['tracked_index'] - 100
    })

    return output_path






def compile_path_tracker(
        par = {},
        target_template   = None,
        folder            = None,
        samples_in_buffer = 10000
    ):

    """
    Build OpenRTDynamics code for the path tracker

    par               - optional parameters for the model (unused so far)
    target_template   - the openrtdynamics code generation terget template to use
    folder            - the folder to which the generated files are written 
    samples_in_buffer - the size of the buffer storing the samples for the input path    
    """

    dy.clear()
    system = dy.enter_system()

    # time-series for velocity, position, orientation, ...
    input_signals = {}
    input_signals['Ts']           = dy.system_input( dy.DataTypeFloat64(1), name='Ts',           default_value=0.01, title="sampling time [s]")
    input_signals['velocity']     = dy.system_input( dy.DataTypeFloat64(1), name='velocity_',    default_value=1, title="vehicle velocity [m/s]")
    input_signals['velocity_dot'] = dy.system_input( dy.DataTypeFloat64(1), name='velocity_dot', default_value=1, title="vehicle acceleration [m/s^2] (optional)")
    input_signals['x']            = dy.system_input( dy.DataTypeFloat64(1), name='x',            default_value=0, title="state x [m]")
    input_signals['y']            = dy.system_input( dy.DataTypeFloat64(1), name='y',            default_value=0, title="state y [m]")
    input_signals['x_dot']        = dy.system_input( dy.DataTypeFloat64(1), name='x_dot',        default_value=0, title="state d/dt x [m/s] (optional)")
    input_signals['y_dot']        = dy.system_input( dy.DataTypeFloat64(1), name='y_dot',        default_value=0, title="state d/dt y [m/s] (optional)")
    input_signals['Delta_u']      = dy.system_input( dy.DataTypeFloat64(1), name='Delta_u',      default_value=0, title="Delta_u [rad] (optional)")
    input_signals['Delta_u_dot']  = dy.system_input( dy.DataTypeFloat64(1), name='Delta_u_dot',  default_value=0, title="Delta_u_dot[rad/s] (optional)")

    # control inputs
    async_input_data_valid = dy.system_input( dy.DataTypeBoolean(1), name='async_input_data_valid')
    input_sample_valid     = dy.system_input( dy.DataTypeBoolean(1), name='input_sample_valid')

    # inputs for asynchronously arriving path samples
    path_sample = {}
    path_sample['d']   = dy.system_input( dy.DataTypeFloat64(1), name='d_sample')
    path_sample['x']   = dy.system_input( dy.DataTypeFloat64(1), name='x_sample')
    path_sample['y']   = dy.system_input( dy.DataTypeFloat64(1), name='y_sample')
    path_sample['psi'] = dy.system_input( dy.DataTypeFloat64(1), name='psi_sample')
    path_sample['K']   = dy.system_input( dy.DataTypeFloat64(1), name='K_sample')

    #
    # combine all input signals in a structure that serve as parameters to the 
    # callback function (the embedded system) vl.path_lateral_modification2
    #


    output_signals = async_path_data_handler(
        input_sample_valid,
        async_input_data_valid, 
        path_sample, 
        path_tracking,
        input_signals,
        par,
        samples_in_buffer
    )


    #
    # outputs
    #

    dy.append_output(output_signals['output_valid'],                   'output_valid')
    dy.append_output(output_signals['need_more_path_input_data'],      'need_more_path_input_data')
    dy.append_output(output_signals['distance_at_the_end_of_horizon'], 'distance_at_the_end_of_horizon')
    dy.append_output(output_signals['distance_ahead'],                 'distance_ahead')
    dy.append_output(output_signals['head_index'],                     'head_index')
    dy.append_output(output_signals['read_position'],                  'read_position')
    dy.append_output(output_signals['elements_free_to_write'],         'elements_free_to_write')

    # data sampled at the closest point on path
    dy.append_output(output_signals['tracked_index'],                  'tracked_index')
    dy.append_output(output_signals['d_star'],                         'd_star')
    dy.append_output(output_signals['v_star'],                         'v_star')

    dy.append_output(output_signals['x_r'],          'x_r')
    dy.append_output(output_signals['y_r'],          'y_r')
    dy.append_output(output_signals['psi_r'],        'psi_r')
    dy.append_output(output_signals['K_r'],          'K_r')

    dy.append_output(output_signals['psi_r_dot'],    'psi_r_dot')

    dy.append_output(output_signals['Delta_l'],      'Delta_l')
    dy.append_output(output_signals['Delta_l_dot'],  'Delta_l_dot')


    # generate code
    if target_template is None:
        target_template = tg.TargetCppMinimal()

    code_gen_results = dy.generate_code(
        template = target_template,
        folder   = folder
    )

    compiled_system = dyexe.CompiledCode(code_gen_results)

    return code_gen_results, compiled_system



