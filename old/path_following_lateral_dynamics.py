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

velocity                = dy.system_input( dy.DataTypeFloat64(1), name='velocity',                  default_value=6.0,  value_range=[0, 25],    title="vehicle velocity")
max_lateral_velocity    = dy.system_input( dy.DataTypeFloat64(1), name='max_lateral_velocity',      default_value=1.0,  value_range=[0, 4.0],   title="maximal lateral velocity")
max_lateral_accleration = dy.system_input( dy.DataTypeFloat64(1), name='max_lateral_accleration',   default_value=2.0,  value_range=[1.0, 4.0], title="maximal lateral acceleration")

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
d_star, x_r, y_r, psi_rr, K_r, Delta_l, tracked_index, Delta_index = track_projection_on_path(path, x, y)

#
# project the vehicle velocity onto the path yielding v_star 
#
# Used formula inside project_velocity_on_path:
#   v_star = d d_star / dt = v * cos( Delta_u ) / ( 1 - Delta_l * K(d_star) ) 
#

Delta_u = dy.signal() # feedback from control
v_star = project_velocity_on_path(velocity, Delta_u, Delta_l, K_r)

dy.append_output(v_star,     'v_star')

#
# compute an enhanced (less noise) signal for the path orientation psi_r by integrating the 
# curvature profile and fusing the result with psi_rr to mitigate the integration drift.
#

psi_r, psi_r_dot = compute_path_orientation_from_curvature( Ts, v_star, psi_rr, K_r, L=1.0 )

dy.append_output(psi_rr,    'psi_rr')
dy.append_output(psi_r_dot, 'psi_r_dot')






#
# lateral open-loop control to realize an 'obstacle-avoiding maneuver'
#
# the dynamic model for the lateral distance Delta_l is 
#
#   d/dt Delta_l = u, 
#
# meaning u is the lateral velocity to which is used to control the lateral
# distance to the path.
#

# generate a velocity profile
u_move_left  = dy.signal_step( dy.int32(50) )  - dy.signal_step( dy.int32(200) )
u_move_right = dy.signal_step( dy.int32(500) ) - dy.signal_step( dy.int32(350) )

# apply a rate limiter to limit the acceleration
u = dy.rate_limit( max_lateral_velocity * (u_move_left + u_move_right), Ts, dy.float64(-1) * max_lateral_accleration, max_lateral_accleration) 

dy.append_output(u, 'u')

# internal lateral model (to verify the lateral dynamics of the simulated vehicle)
Delta_l_mdl = dy.euler_integrator(u, Ts)
dy.append_output(Delta_l_mdl, 'Delta_l_mdl')





#
# path tracking control
#
# Control of the lateral distance to the path can be performed via the augmented control
# variable u. 
#
# Herein, a linearization yielding the resulting lateral dynamics u --> Delta_l : 1/s is applied.
#

Delta_u << dy.asin( dy.saturate(u / velocity, -0.99, 0.99) )
delta_star = psi_r - psi
delta =  delta_star + Delta_u
delta = dy.unwrap_angle(angle=delta, normalize_around_zero = True)

dy.append_output(Delta_u, 'Delta_u')
dy.append_output(delta_star, 'delta_star')


#
# The model of the vehicle including a disturbance
#


# steering angle limit
delta = dy.saturate(u=delta, lower_limit=-math.pi/2.0, upper_limit=math.pi/2.0)

# the model of the vehicle
x_, y_, psi_, x_dot, y_dot, psi_dot = discrete_time_bicycle_model(delta, velocity, Ts, wheelbase)

# close the feedback loops
x << x_
y << y_
psi << psi_



#
# outputs: these are available for visualization in the html set-up
#

dy.append_output(x, 'x')
dy.append_output(y, 'y')
dy.append_output(psi, 'psi')

dy.append_output(delta, 'steering')

dy.append_output(x_r, 'x_r')
dy.append_output(y_r, 'y_r')
dy.append_output(psi_r, 'psi_r')

dy.append_output(Delta_l, 'Delta_l')




# generate code for Web Assembly (wasm), requires emcc (emscripten) to build
code_gen_results = dy.generate_code(template=dy.TargetWasm(enable_tracing=False), folder="generated/path_following_lateral_dynamics", build=True)

#
dy.clear()

