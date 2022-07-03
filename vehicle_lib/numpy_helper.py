import numpy as np
import math


#
# helper functions functions for Numpy calculations 
#

def co(time, val, Ts=1.0):
    return val * np.ones(int(math.ceil(time / Ts)))

def cosra(time, val1, val2, Ts=1.0):
    N = int(math.ceil(time / Ts))
    return val1 + (val2-val1) * (0.5 + 0.5 * np.sin(math.pi * np.linspace(0, 1, N) - math.pi/2))

def ra(time, val1, val2, Ts=1.0):
    N = int(math.ceil(time / Ts))
    return np.linspace(val1, val2, N)

def filtfilt(Ts, input_seq, cutoff_frq = 0.0325):
    
    b, a = signal.butter(1, cutoff_frq)
    output_seq = signal.filtfilt(b, a, input_seq, padlen=150)
    
    return output_seq


def numerical_derivative(Ts, u, cutoff_frq = 0.0325):
    u_dot = np.diff( u  ) / Ts

    u_dot_filtered = filtfilt(Ts, u_dot, cutoff_frq)
    
    u_dot_filtered = np.concatenate(( u_dot_filtered , [ u_dot_filtered[-1] ] ))

    return u_dot_filtered

def make_time(Ts, seq):
    N = np.size(seq)
    time_seq = np.linspace(0, Ts*(N-1), N)
    
    # test: np.diff(make_time(Ts, Delta_l_r_profile)) == Ts
    
    return time_seq

def np_normalize_angle_mpi_to_pi(angle : np.array) -> np.array:
    
    # test: should be [10.0, -80.0, 178.0, -178.0]
    # a = np_normalize_angle_mpi_to_pi( np.deg2rad( np.array([  10.0, -80.0, -182, 182 ]) ) )
    # np.rad2deg(a).tolist()    

    angle_out = angle.copy()
    
    I_too_small = np.where( angle < np.deg2rad(-180) )[0]
    angle_out[I_too_small] = angle[I_too_small] + np.deg2rad(360)
    
    I_too_big = np.where( angle > np.deg2rad(180) )[0]
    angle_out[I_too_big] = angle[I_too_big] - np.deg2rad(360)

    return angle_out

def rotate_vector_2d(alpha, xy):
    # test
    # rotmat2d( alpha = np.deg2rad(-45), xy=np.array([1.0, 0.0]) )
    xy_rotated = np.array([ 
            [ np.cos(alpha), -np.sin(alpha) ],
            [ np.sin(alpha),  np.cos(alpha) ]
        ]) @ xy
    
    return xy_rotated

