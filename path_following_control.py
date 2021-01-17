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
# Demo: a vehicle controlled to follow a given path
#
#       Implemented using the code generator openrtdynamics 2 - https://pypi.org/project/openrtdynamics2/ .
#       This generates c++ code for Web Assembly to be run within the browser.
#

system = dy.enter_system()

velocity               = dy.system_input( dy.DataTypeFloat64(1), name='velocity',              default_value=23.75,  value_range=[0, 25],   title="vehicle velocity")
k_p                    = dy.system_input( dy.DataTypeFloat64(1), name='k_p',                   default_value=2.0,    value_range=[0, 10.0], title="controller gain")
disturbance_amplitude  = dy.system_input( dy.DataTypeFloat64(1), name='disturbance_amplitude', default_value=20.0,   value_range=[-45, 45], title="disturbance amplitude") * dy.float64(math.pi / 180.0)
sample_disturbance     = dy.system_input( dy.DataTypeInt32(1),   name='sample_disturbance',    default_value=50,     value_range=[0, 300],  title="disturbance position")

# parameters
wheelbase = 3.0

# sampling time
Ts = 0.01

# create storage for the reference path:
path = import_path_data(track_data)

# create placeholders for the plant output signals
x   = dy.signal()
y   = dy.signal()
psi = dy.signal()

# track the evolution of the closest point on the path to the vehicles position
d_star, x_r, y_r, psi_r, K_r, Delta_l, tracked_index, Delta_index = track_projection_on_path(path, x, y)

# reference for the lateral distance
Delta_l_r = dy.float64(0.0) # zero in this example

dy.append_primay_ouput(Delta_l_r, 'Delta_l_r')

# feedback control
u = dy.PID_controller(r=Delta_l_r, y=Delta_l, Ts=0.01, kp=k_p)

# path tracking
# resulting lateral model u --> Delta_l : 1/s
Delta_u = dy.asin( dy.saturate(u / velocity, -0.99, 0.99) )
delta_star = psi_r - psi
steering =  delta_star + Delta_u
steering = dy.unwrap_angle(angle=steering, normalize_around_zero = True)

dy.append_primay_ouput(Delta_u, 'Delta_u')
dy.append_primay_ouput(delta_star, 'delta_star')


#
# The model of the vehicle including a disturbance
#

# model the disturbance
disturbance_transient = np.concatenate(( cosra(50, 0, 1.0), co(10, 1.0), cosra(50, 1.0, 0) ))
steering_disturbance, i = dy.play(disturbance_transient, start_trigger=dy.counter() == sample_disturbance, auto_start=False)

# apply disturbance to the steering input
disturbed_steering = steering + steering_disturbance * disturbance_amplitude

# steering angle limit
disturbed_steering = dy.saturate(u=disturbed_steering, lower_limit=-math.pi/2.0, uppper_limit=math.pi/2.0)

# the model of the vehicle
x_, y_, psi_, x_dot, y_dot, psi_dot = discrete_time_bicycle_model(disturbed_steering, velocity, Ts, wheelbase)

# close the feedback loops
x << x_
y << y_
psi << psi_



#
# outputs: these are available for visualization in the html set-up
#

dy.append_primay_ouput(x, 'x')
dy.append_primay_ouput(y, 'y')
dy.append_primay_ouput(psi, 'psi')

dy.append_primay_ouput(steering, 'steering')

dy.append_primay_ouput(x_r, 'x_r')
dy.append_primay_ouput(y_r, 'y_r')
dy.append_primay_ouput(psi_r, 'psi_r')

dy.append_primay_ouput(Delta_l, 'Delta_l')

dy.append_primay_ouput(steering_disturbance, 'steering_disturbance')
dy.append_primay_ouput(disturbed_steering, 'disturbed_steering')

dy.append_primay_ouput(tracked_index, 'tracked_index')
dy.append_primay_ouput(Delta_index, 'Delta_index')


# generate code for Web Assembly (wasm), requires emcc (emscripten) to build
code_gen_results = dy.generate_code(template=dy.WasmRuntime(enable_tracing=False), folder="generated/path_following_control", build=True)

#
dy.clear()
