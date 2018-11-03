from sympy import *

x = Symbol('x')
f = Function('f')
x0 = Symbol('x0')
y0 = Symbol('y0')
f = sqrt((x0-x)**2 + (y0 - log(x+2) + 6 )**2)

print(diff(f, x))
