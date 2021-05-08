import math
import numpy as np
import matplotlib.pyplot as plt

import openrtdynamics2.lang as dy
from openrtdynamics2.ORTDtoNumpy import ORTDtoNumpy

from .vehicle_lib import make_time




@ORTDtoNumpy()
def generate_one_dimensional_motion_trajectory(acceleration_input, Ts):

    # Delta_l_dotdot; the motion jerk is limited to +-1.0 m/s^3 
    acceleration     = dy.rate_limit(acceleration_input, Ts, lower_limit = -1.0, upper_limit = 1.0)

    # Delta_l_dot        
    velocity         = dy.euler_integrator(acceleration, Ts, 0)

    # Delta_l
    lateral_distance = dy.euler_integrator(velocity,     Ts, 0)

    return lateral_distance, velocity, acceleration



def generate_lateral_profile_1(Ts, T_phase1=1, T_phase2=3, T_phase3=1, T_phase4=3, T_phase5=1):
    """
        Generate a one-dimensional motion trajectory for a jerk-limited sidewards movement.
        
    """
    
    def ones_by_time(T):
        return np.ones( int(math.floor( T / Ts )) )


    
    lateral_distance, velocity, acceleration = generate_one_dimensional_motion_trajectory(
        np.concatenate((
            
            # phase 1: no movement 
            +0.0 * ones_by_time(T_phase1),
            
            # phase 2: movement into the positive direction
            #          divided into an accelerating phase,
            #          a phase of constant velocity, and finally
            #          a braking phase. The velocity is then zero.
            +1.0 * ones_by_time(T_phase2/3),
            +0.0 * ones_by_time(T_phase2/3),
            -1.0 * ones_by_time(T_phase2/3),
            
            # phase 3: no movement 
            +0.0 * ones_by_time(T_phase3),
            
            # phase 4: like phase 2, however, in the opposite direction 
            -1.0 * ones_by_time(T_phase4/3),
            +0.0 * ones_by_time(T_phase4/3),
            +1.0 * ones_by_time(T_phase4/3),
            
            # phase 5: no movement 
            +0.0 * ones_by_time(T_phase5)

        )),
        0.01
    )

    lateral_profile = {}

    lateral_profile['Ts'] = Ts

    lateral_profile['Delta_l_r_dotdot'] = acceleration
    lateral_profile['Delta_l_r_dot'] = velocity
    lateral_profile['Delta_l_r'] = lateral_distance

    # velocity of the vehicle
    lateral_profile['v'] = 5 * np.ones( len( lateral_profile['Delta_l_r'] ) )
    
    return lateral_profile

def plot_lateral_profile( lateral_profile ):
    """
        Show a profile for the lateral distance to the original path 
    """
    
    time = make_time(lateral_profile['Ts'], lateral_profile['Delta_l_r'])
    plt.figure(figsize=(12,3), dpi=100)
    plt.plot(time, lateral_profile['Delta_l_r'] )
    plt.plot(time, lateral_profile['Delta_l_r_dot'] )
    plt.plot(time, lateral_profile['Delta_l_r_dotdot'] )
    plt.legend(['intended lateral distance [$m$]', '1st derivative [$m/s$]', '2nd derivative [$m/s^s$]'])
    plt.xlabel('time [s]')
    plt.ylabel('lateral distance, velocity, acceleration')
    plt.show()

