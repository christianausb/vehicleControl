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

velocity               = dy.system_input( dy.DataTypeFloat64(1), name='velocity',              default_value=25.0,   value_range=[0, 25],     title="vehicle velocity [m/s]")
steering_rate          = dy.system_input( dy.DataTypeFloat64(1), name='steering_rate',         default_value=1.0,    value_range=[-10, 10],   title="steering_rate [degrees/s]")              * dy.float64(math.pi / 180.0)
initial_steering       = dy.system_input( dy.DataTypeFloat64(1), name='initial_steering',      default_value=-10.0,  value_range=[-40, 40],   title="initial steering angle [degrees]")       * dy.float64(math.pi / 180.0)
initial_orientation    = dy.system_input( dy.DataTypeFloat64(1), name='initial_orientation',   default_value=0.0,    value_range=[-360, 360], title="initial orientation angle [degrees]")    * dy.float64(math.pi / 180.0)

# parameters
wheelbase = 3.0

# sampling time
Ts = 0.01

# create storage for the reference path:
path = import_path_data(track_data)

# linearly increasing steering angle
delta = dy.euler_integrator( steering_rate, Ts, initial_state=initial_steering )
delta = dy.saturate(u=delta, lower_limit=-math.pi/2.0, uppper_limit=math.pi/2.0)

# the model of the vehicle
x, y, psi, x_dot, y_dot, psi_dot = discrete_time_bicycle_model(delta, velocity, Ts, wheelbase, psi0=initial_orientation)


#
# outputs: these are available for visualization in the html set-up
#

dy.append_primay_ouput(x, 'x')
dy.append_primay_ouput(y, 'y')
dy.append_primay_ouput(psi, 'psi')
dy.append_primay_ouput(delta, 'steering')

# generate code for Web Assembly (wasm), requires emcc (emscripten) to build
code_gen_results = dy.generate_code(template=dy.WasmRuntime(enable_tracing=False), folder="generated/bicycle_model", build=True)

#
dy.clear()
