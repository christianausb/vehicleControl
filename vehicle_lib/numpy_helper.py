import numpy as np



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
