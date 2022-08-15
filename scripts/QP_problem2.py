from scipy.optimize import minimize, rosen, rosen_der
import numpy as np
W = np.array([[1,0,0],[0,1,0],[0,0,1]])
fun = lambda f: np.matmul(np.transpose(f),W)

cons = ({'type': 'ineq', 'fun': lambda x:  x[0] - 2 * x[1] + 2},
        {'type': 'ineq', 'fun': lambda x: -x[0] - 2 * x[1] + 6},
        {'type': 'ineq', 'fun': lambda x: -x[0] + 2 * x[1] + 2})

bnds = ((0, None), (0, None))

res = minimize(fun, (2, 0), method='SLSQP', bounds=bnds)
print(res.x)