import math
import numpy as np

import openrtdynamics2.lang as dy
import openrtdynamics2.py_execute as dyexe

import vehicle_lib.vehicle_lib as vl



def async_path_data_handler(
        input_sample_valid, 
        async_input_data_valid, 
        path_sample, 
        input_signals,
        d0, x0, y0, psi0, delta0, delta_dot0,
        par = {},
        samples_in_buffer = 10000
    ):
    
    # allocate the buffers to collect the input data
    path = vl.create_path_horizon( samples_in_buffer )
    
    # write new data into the buffer as valid samples arrive 
    with dy.sub_if(async_input_data_valid, subsystem_name='store_input_data') as system:
        
        vl.append_to_path_horizon(path, path_sample)

        
    #
    # run the controller in case this is requested via 'input_sample_valid'
    #
    
    with dy.sub_if(input_sample_valid, subsystem_name='process_data', prevent_output_computation=True) as system:

        Ts, wheelbase, velocity, Delta_l_r, Delta_l_r_dot, Delta_l_r_dotdot = input_signals

        output_signals = vl.path_lateral_modification2(
            Ts,
            wheelbase,
            path,
            velocity,
            Delta_l_r,
            Delta_l_r_dot,
            Delta_l_r_dotdot,
            d0, x0, y0, psi0, delta0, delta_dot0,
            par
        )
        
        
        system.set_outputs( output_signals.to_list() )
    output_signals.replace_signals( system.outputs )
    
    
    # is the controller executed and yielded valid outputs?
    control_variables_valid = dy.logic_and( input_sample_valid, output_signals['output_valid'] )
    
    # sample & hold of some output signals that are needed
    output_signals['d_star'] = dy.sample_and_hold(
        output_signals['d_star'], 
        control_variables_valid, 
        initial_state = 0 
    )
    
    output_signals['minimal_read_position'] = dy.sample_and_hold(
        output_signals['minimal_read_position'], 
        control_variables_valid, 
        initial_state = 0 
    )
    

    # get the length of the horizon
    output_signals['head_index'], output_signals['distance_at_the_end_of_horizon'] = vl.path_horizon_head_index(path)


    # distance_at_the_end_of_horizon = rb.read_from_absolute_index(reference['time'], head_index)
    output_signals['distance_ahead'] = output_signals['distance_at_the_end_of_horizon'] - output_signals['d_star']


    # compute the number of elements in the circular buffer that are free to write
    output_signals['elements_free_to_write'] = samples_in_buffer - ( output_signals['head_index'] - output_signals['minimal_read_position'] + 1 )

    
    
    return output_signals


def compile_lateral_path_transformer(
        wheelbase = 3.0, 
        Ts = 0.01, 
        par = {}
    ):

    """
    Build OpenRTDynamics code for the lateral path transformation
    """

    dy.clear()
    system = dy.enter_system()

    # time-series for velocity, lateral distance, ...
    velocity               = dy.system_input( dy.DataTypeFloat64(1), name='velocity_',         default_value=1,      value_range=[0, 25],   title="vehicle velocity")
    Delta_l_r              = dy.system_input( dy.DataTypeFloat64(1), name='Delta_l_r',         default_value=0.0,    value_range=[-10, 10], title="lateral deviation to the path")
    Delta_l_r_dot          = dy.system_input( dy.DataTypeFloat64(1), name='Delta_l_r_dot',     default_value=0.0,    value_range=[-10, 10], title="1st-order time derivative of lateral deviation to the path")
    Delta_l_r_dotdot       = dy.system_input( dy.DataTypeFloat64(1), name='Delta_l_r_dotdot',  default_value=0.0,    value_range=[-10, 10], title="2nd-order time derivative of lateral deviation to the path")

    # initial states of the vehicle
    d0         = dy.system_input( dy.DataTypeFloat64(1), name='d0',         default_value=0, title="initial state d0")
    x0         = dy.system_input( dy.DataTypeFloat64(1), name='x0',         default_value=0, title="initial state x0")
    y0         = dy.system_input( dy.DataTypeFloat64(1), name='y0',         default_value=0, title="initial state y0")
    psi0       = dy.system_input( dy.DataTypeFloat64(1), name='psi0',       default_value=0, title="initial state psi0")
    delta0     = dy.system_input( dy.DataTypeFloat64(1), name='delta0',     default_value=0, title="initial state delta0")
    delta_dot0 = dy.system_input( dy.DataTypeFloat64(1), name='delta_dot0', default_value=0, title="initial state delta_dot0")

    # control inputs
    async_input_data_valid = dy.system_input( dy.DataTypeBoolean(1), name='async_input_data_valid')
    input_sample_valid     = dy.system_input( dy.DataTypeBoolean(1), name='input_sample_valid')

    # async path samples
    path_sample = {}
    path_sample['d']   = dy.system_input( dy.DataTypeFloat64(1), name='d_sample')
    path_sample['x']   = dy.system_input( dy.DataTypeFloat64(1), name='x_sample')
    path_sample['y']   = dy.system_input( dy.DataTypeFloat64(1), name='y_sample')
    path_sample['psi'] = dy.system_input( dy.DataTypeFloat64(1), name='psi_sample')
    path_sample['K']   = dy.system_input( dy.DataTypeFloat64(1), name='K_sample')



    input_signals = Ts, wheelbase, velocity, Delta_l_r, Delta_l_r_dot, Delta_l_r_dotdot


    output_signals = async_path_data_handler(
        input_sample_valid,
        async_input_data_valid, 
        path_sample, 
        input_signals,
        d0, x0, y0, psi0, delta0, delta_dot0,
        par,
    )


    #
    # outputs: these are available for visualization in the html set-up
    #



    dy.append_output(output_signals['output_valid'],                   'output_valid')
    dy.append_output(output_signals['need_more_path_input_data'],      'need_more_path_input_data')
    dy.append_output(output_signals['distance_at_the_end_of_horizon'], 'distance_at_the_end_of_horizon')
    dy.append_output(output_signals['distance_ahead'],                 'distance_ahead')
    dy.append_output(output_signals['head_index'],                     'head_index')
    dy.append_output(output_signals['read_position'],                  'read_position')
    dy.append_output(output_signals['elements_free_to_write'],         'elements_free_to_write')

    dy.append_output(output_signals['tracked_index'],                  'tracked_index')
    dy.append_output(output_signals['d_star'],                         'path_d_star')

    dy.append_output(output_signals['d'],     'path_d')
    dy.append_output(output_signals['x'],     'path_x')
    dy.append_output(output_signals['y'],     'path_y')
    dy.append_output(output_signals['psi_r'], 'path_psi')
    dy.append_output(output_signals['K'],     'path_K')

    dy.append_output(output_signals['delta'],         'vehicle_delta')
    dy.append_output(output_signals['delta_dot'],     'vehicle_delta_dot')

    dy.append_output(output_signals['psi'],           'vehicle_psi')
    dy.append_output(output_signals['psi_dot'],       'vehicle_psi_dot')

    dy.append_output(velocity*dy.float64(1.0),        'velocity')


    # generate code
    code_gen_results = dy.generate_code(
        template=dy.TargetRawCpp(enable_tracing=False)
    )

    compiled_system = dyexe.CompiledCode(code_gen_results)

    return code_gen_results, compiled_system








def put_input_path_sample(output_data, input_data, raw_cpp_instance, path, index):

    if type(path) is dict:

        if index >= len(path['D']):
            # reached_end = False
            return True
            
        input_data.d_sample    = path['D'][index]
        input_data.x_sample    = path['X'][index]
        input_data.y_sample    = path['Y'][index]
        input_data.psi_sample  = path['PSI'][index]
        input_data.K_sample    = path['K'][index]


    elif callable(path):
        # run callback that puts data into 'input_data'
        reached_to_end = path(input_data, index)

        if reached_to_end:
            return True


    # set flags indicating path input data only    
    input_data.input_sample_valid     = False # do not trigger the controller
    input_data.async_input_data_valid = True  # put new path data

    raw_cpp_instance.step(output_data, input_data, True, False, False)

    # update (does not change output_data)
    raw_cpp_instance.step(output_data, input_data, False, True, False)

    if output_data.elements_free_to_write < 1:
        return True
    
    return False

        
def execute_control_step(output_data, input_data, raw_cpp_instance, input_signals, index):

    input_data.input_sample_valid     = True  # do not trigger the controller
    input_data.async_input_data_valid = False # put new path data

    input_data.velocity_          = input_signals['v'][index]
    input_data.Delta_l_r          = input_signals['Delta_l_r'][index]
    input_data.Delta_l_r_dot      = input_signals['Delta_l_r_dot'][index]
    input_data.Delta_l_r_dotdot   = input_signals['Delta_l_r_dotdot'][index]
    
    #
    raw_cpp_instance.step(output_data, input_data, True, False, False)

    # update (does not change output_data)
    raw_cpp_instance.step(output_data, input_data, False, True, False)

        
def set_initial_states( input_data, initial_states={} ):

    # default values
    input_data.d0         = 0.0
    input_data.x0         = 0.0
    input_data.y0         = 0.0
    input_data.psi0       = 0.0
    input_data.delta0     = 0.0
    input_data.delta_dot0 = 0.0

    # overwrites
    for key, value in initial_states.items():
        setattr(input_data, key, value)






def run_lateral_path_transformer(
        input_data, 
        output_data, 
        raw_cpp_instance, 
        input_path, 
        input_sequence,
        initial_states={}
    ):


    # simulate n steps
    n = len( input_sequence['Delta_l_r'] )-1

    #n = 20

    # storage for the output data
    path = {}
    path['D']   = math.nan * np.zeros(n)
    path['X']   = math.nan * np.zeros(n)
    path['Y']   = math.nan * np.zeros(n)
    path['PSI'] = math.nan * np.zeros(n)
    path['K']   = math.nan * np.zeros(n)

    path['D_STAR']   = math.nan * np.zeros(n)


    path['V_DELTA_DOT']   = math.nan * np.zeros(n)
    path['V_DELTA']       = math.nan * np.zeros(n)
    path['V_PSI_DOT']     = math.nan * np.zeros(n)
    path['V_PSI']         = math.nan * np.zeros(n)

    path['V_VELOCITY']    = math.nan * np.zeros(n)


    distance_at_the_end_of_horizon  = []
    distance_ahead  = []
    head_index  = []
    read_position  = []
    elements_free_to_write  = []
    tracked_index = []


    input_data_read_index = 0

    # set initial states via the input signals
    set_initial_states( input_data, initial_states ) 

    # reset the states of the system
    raw_cpp_instance.step(output_data, input_data, False, False, True)

    # pre-fill path horizon; could be omitted, however, then the controller will ask for more data by itself 
    for i in range(0,10):
#    while True:
    
        reached_end = put_input_path_sample( output_data, input_data, raw_cpp_instance, input_path, input_data_read_index )
        if reached_end:
            #print('buffer filled')
            break

        input_data_read_index += 1

    # run loop
    reached_end = False

    for i in range( 0, n ):

        while True:
        
            # run controller and see if enough path data is available 
            execute_control_step(
                output_data,
                input_data,
                raw_cpp_instance,
                input_sequence,
                i
            )
            

            if output_data.output_valid == False:
                
                # another path input sample is needed
                reached_end = put_input_path_sample( output_data, input_data, raw_cpp_instance, input_path, input_data_read_index )
                if reached_end:
                    break

                input_data_read_index += 1
                

                # watch out to not cause an buffer overflow by passing too much data that cannot be
                # consumed in time! (not checked here)
                if output_data.elements_free_to_write < 10:

                    raise BaseException('..')
                
                
                continue
                
            else:
                # controller yielded new control variables
                break

        
        if reached_end:
            print("reached end of input path")

            break

        # output_data contains valid control variables
        distance_at_the_end_of_horizon.append( output_data.distance_at_the_end_of_horizon )
        distance_ahead.append(                 output_data.distance_ahead )
        head_index.append(                     output_data.head_index  )
        read_position.append(                  output_data.read_position )
        elements_free_to_write.append(         output_data.elements_free_to_write )

        tracked_index.append(                  output_data.tracked_index )
            

        path['D'][i]      = output_data.path_d
        path['X'][i]      = output_data.path_x
        path['Y'][i]      = output_data.path_y
        path['PSI'][i]    = output_data.path_psi
        path['K'][i]      = output_data.path_K
        
        path['D_STAR'][i] = output_data.path_d_star
        
        path['V_DELTA_DOT'][i]   = output_data.vehicle_delta_dot
        path['V_DELTA'][i]       = output_data.vehicle_delta
        path['V_PSI_DOT'][i]     = output_data.vehicle_psi
        path['V_PSI'][i]         = output_data.vehicle_psi_dot

        path['V_VELOCITY'][i]    = output_data.velocity

    return path




class LateralPathTransformer():

    def __init__(self, wheelbase, par={}):

        self.code_gen_results, self.compiled_system = compile_lateral_path_transformer(
            wheelbase = wheelbase, 
            Ts        = 0.01,
            par       = par
        )


        # Create an instance of the system (this is an instance of the c++ class wrapped by cppyy)
        self._raw_cpp_instance = self.compiled_system.system_class()

        # create data strutures to store I/O data
        self._input_data  = self.compiled_system.system_class.Inputs()
        self._output_data = self.compiled_system.system_class.Outputs()

    def run_lateral_path_transformer( self, input_path, lateral_profile, initial_states={} ):

        return run_lateral_path_transformer(
            input_data        = self._input_data, 
            output_data       = self._output_data,
            raw_cpp_instance  = self._raw_cpp_instance,
            input_path        = input_path,
            input_sequence    = lateral_profile,
            initial_states    = initial_states
        )
