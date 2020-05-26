from scipy.optimize import fsolve
import math

def f(z):
    x,y = z
    f1 = (x-5)**2 + (y-5)**2 - 1
    f2 = -x - y + 1
    return [f1,f2]

print(f([3.685,3.688]))

x = fsolve(f,[1,1])
print(x)