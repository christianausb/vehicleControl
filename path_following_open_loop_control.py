from openrtdynamics2.dsp import *
import json
import math
import numpy as np
import openrtdynamics2.lang as dy
import os

from vehicle_lib.vehicle_lib import *

# load track data
with open("track_data/simple_track.json", "r") as read_file:
    track_data = json.load(read_file)


#
# Demo: a vehicle controlled in an open-loop manner to follow a given path as long as possible
#
#       Implemented using the code generator openrtdynamics 2 - https://pypi.org/project/openrtdynamics2/ .
#       This generates c++ code for Web Assembly to be run within the browser.
#

system = dy.enter_system()

initial_velocity       = dy.system_input( dy.DataTypeFloat64(1), name='velocity',              default_value=11.0,   value_range=[0, 25],     title="initial condition: vehicle velocity")
acceleration           = dy.system_input( dy.DataTypeFloat64(1), name='acceleration',          default_value=-2.5,   value_range=[-8, 0],     title="initial condition: vehicle acceleration")
disturbance_ofs        = dy.system_input( dy.DataTypeFloat64(1), name='disturbance_ofs',       default_value=-0.7,   value_range=[-4, 4],     title="disturbance: steering offset") * dy.float64(math.pi / 180.0)
delta_factor           = dy.system_input( dy.DataTypeFloat64(1), name='delta_factor',          default_value=1.1,    value_range=[0.8, 1.2],  title="disturbance: steering factor")
velocity_factor        = dy.system_input( dy.DataTypeFloat64(1), name='velocity_factor',       default_value=1.0,    value_range=[0.8, 1.2],  title="disturbance: velocity factor")
IMU_drift              = dy.system_input( dy.DataTypeFloat64(1), name='IMU_drift',             default_value=0.0,    value_range=[-0.5, 0.5], title="disturbance: drift of orientation angle [degrees/s]") * dy.float64(math.pi / 180.0)
activate_IMU           = dy.system_input( dy.DataTypeBoolean(1), name='activate_IMU',          default_value=0,      value_range=[0, 1],      title="mode: activate IMU")

# parameters
wheelbase = 3.0

# sampling time
Ts = 0.01

# create storage for the reference path:
path = import_path_data(track_data)

# create placeholders for the plant output signals
x_real   = dy.signal()
y_real   = dy.signal()
psi_measurement = dy.signal()

#
# track the evolution of the closest point on the path to the vehicles position
# note: this is only used to initialize the open-loop control with the currect vehicle position on the path
#
d_star, x_r, y_r, psi_r, K_r, Delta_l, tracked_index, Delta_index = track_projection_on_path(path, x_real, y_real)

path_index_start_open_loop_control = dy.sample_and_hold(tracked_index, event=dy.initial_event())
path_distance_start_open_loop_control = dy.sample_and_hold(d_star, event=dy.initial_event())

dy.append_primay_ouput(path_index_start_open_loop_control,    'path_index_start_open_loop_control')
dy.append_primay_ouput(Delta_l, 'Delta_l')


#
# model vehicle braking
#

velocity = dy.euler_integrator(acceleration, Ts, initial_state=initial_velocity) * velocity_factor
velocity = dy.saturate(velocity, lower_limit=0)

#
# open-loop control
#

# estimate the travelled distance by integration of the vehicle velocity
d_hat = dy.euler_integrator(velocity, Ts, initial_state=0) + path_distance_start_open_loop_control

# estimated travelled distance (d_hat) to path-index 
open_loop_index, _, _ = tracker_distance_ahead(path, current_index=path_index_start_open_loop_control, distance_ahead=d_hat)

dy.append_primay_ouput(open_loop_index,    'open_loop_index')

# get the reference orientation and curvature
_, _, _, psi_rr, K_r = sample_path(path, index=open_loop_index + dy.int32(1) )  # new sampling

#
# compute an enhanced (less noise) signal for the path orientation psi_r by integrating the 
# curvature profile and fusing the result with psi_rr to mitigate the integration drift.
#

psi_r, psi_r_dot = compute_path_orientation_from_curvature( Ts, velocity, psi_rr, K_r, L=1.0 )

dy.append_primay_ouput(psi_rr,    'psi_rr')
dy.append_primay_ouput(psi_r_dot, 'psi_r_dot')


# feedback of internal model
psi_mdl = dy.signal()

# switch between IMU feedback and internal model
psi_feedback = psi_mdl
psi_feedback = dy.conditional_overwrite( psi_feedback, activate_IMU, psi_measurement )

# path tracking
Delta_u = dy.float64(0.0)
steering =  psi_r - psi_feedback + Delta_u
steering = dy.unwrap_angle(angle=steering, normalize_around_zero = True)

dy.append_primay_ouput(Delta_u, 'Delta_u')

# internal model of carbody rotation (from bicycle model)
psi_mdl << dy.euler_integrator( velocity * dy.float64(1.0 / wheelbase) * dy.sin(steering), Ts, initial_state=psi_measurement )
dy.append_primay_ouput(psi_mdl, 'psi_mdl')




#
# The model of the vehicle including a disturbance
#

# the model of the vehicle
x_, y_, psi_real, *_ = lateral_vehicle_model(u_delta=steering, v=velocity, v_dot=dy.float64(0), 
                                        Ts=Ts, wheelbase=wheelbase,  
                                        delta_disturbance=disturbance_ofs, 
                                        delta_factor=delta_factor)

#
# error model of orientation angle sensing
#

psi_ofs = dy.euler_integrator(IMU_drift, Ts, initial_state=0)
psi_measurement_ = psi_real + psi_ofs

dy.append_primay_ouput(psi_measurement_, 'psi_measurement')

# close the feedback loops
x_real << x_
y_real << y_
psi_measurement << psi_measurement_

#
# outputs: these are available for visualization in the html set-up
#

dy.append_primay_ouput(x_real, 'x')
dy.append_primay_ouput(y_real, 'y')
dy.append_primay_ouput(psi_real, 'psi_real')

dy.append_primay_ouput(steering, 'steering')

dy.append_primay_ouput(x_r, 'x_r')
dy.append_primay_ouput(y_r, 'y_r')
dy.append_primay_ouput(psi_r, 'psi_r')


dy.append_primay_ouput(tracked_index, 'tracked_index')


# generate code for Web Assembly (wasm), requires emcc (emscripten) to build
sourcecode, manifest = dy.generate_code(template=dy.WasmRuntime(enable_tracing=False), folder="generated/path_following_open_loop_control", build=True)

#
dy.clear()
