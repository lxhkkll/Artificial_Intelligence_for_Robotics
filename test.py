from sympy import *

# x = Symbol('x')
# f = Function('f')
# x0 = Symbol('x0')
# y0 = Symbol('y0')
# f = sqrt((x0-x)**2 + (y0 - log(x+2) + 6 )**2)
#
# print(diff(f, x))


import numpy as np

A = np.arange(2 , 14).reshape((3 , 4))
A[1 , 1] = 8
print('A:' , A)
# A: [[ 2  3  4  5]
#  [ 6  8  8  9]
#  [10 11 12 13]]

print(np.diff(A, n=1,axis=1))
# [[1 1 1]
#  [2 0 1]
#  [1 1 1]]

