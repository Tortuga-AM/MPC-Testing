import tinympc
import numpy as np
import os

# 1. Setup Dynamics
dt = 0.02
# State: [x, y, theta, vx, vy, omega]
A = np.array([
    [1, 0, 0, dt, 0,  0 ],
    [0, 1, 0, 0,  dt, 0 ],
    [0, 0, 1, 0,  0,  dt],
    [0, 0, 0, 1,  0,  0 ],
    [0, 0, 0, 0,  1,  0 ],
    [0, 0, 0, 0,  0,  1 ]
], dtype=np.float64).copy('F')

B = np.array([
    [0.5*dt**2, 0,         0        ],
    [0,         0.5*dt**2, 0        ],
    [0,         0,         0.5*dt**2],
    [dt,        0,         0        ],
    [0,         dt,        0        ],
    [0,         0,         dt       ]
], dtype=np.float64).copy('F')

# ENSURE Q AND R ARE 2D MATRICES
# Using np.eye and multiplying ensures a proper square 2D array
Q = np.eye(6, dtype=np.float64)
Q[0,0], Q[1,1], Q[2,2], Q[3,3], Q[4,4], Q[5,5] = 10, 10, 5, 1, 1, 1
Q = Q.copy('F')

R = np.eye(3, dtype=np.float64)
R[0,0], R[1,1], R[2,2] = 0.1, 0.1, 0.1
R = R.copy('F')

# 2. Initialize and Setup
prob = tinympc.TinyMPC()

# The wrapper version you have seems to want these specific keywords:
# nx: number of states (6)
# nu: number of inputs (3)
# N: horizon (10)
# rho: penalty parameter (1.0)
try:
    prob.setup(
        A=A, 
        B=B, 
        Q=Q, 
        R=R, 
        n_horizon=10, # Some versions use n_horizon
        N=10,         # Some versions use N for horizon
        nx=6, 
        nu=3, 
        rho=1.0, 
        verbose=True
    )
except TypeError:
    # Fallback for older/middle versions that use different keywords
    print("Keyword mismatch, trying alternate setup...")
    prob.setup(A, B, Q, R, n_horizon=10, rho=1.0, verbose=1)

prob.codegen("generated_code", verbose=1)
print(f"Success! Solver generated in {output_path}")