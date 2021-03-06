import numpy as np
from hammerhead_control.msg import Errors
"""
[Reference]
https://www.sfu.ca/~ssurjano/index.html
"""


def calculation(err, int_err, prev_err, array, t):#as you want
    fitness_inverse = error_based_fitness(err, int_err, prev_err, array)
    if fitness_inverse == 0:
       fitness = 0
    else:
       fitness = 1/fitness_inverse
    return fitness

def error_based_fitness(err, int_err, prev_err, param_vals):
    cost = 0 
    terms = np.array([err, int_err, (err - prev_err)])
    cost = np.array(param_vals)*terms
    return abs(np.sum(cost))

"""Benchmark Functions"""
def eggholder(array):
    z = - (array[1] + 47) * np.sin(np.sqrt(abs(array[1] + (array[0]/2) +47))) - array[0] *np.sin(np.sqrt(abs(array[0] - (array[1]+47))))
    return z

def sphere(array):
    fitness = 0
    for i in range(len(array)):
        fitness = fitness + array[i]**2
    return fitness

def rastrigin(array):
    sum = 0
    fitness = 0
    for x in array:
        sum = sum + x**2 - 10 * np.cos(2 * np.pi * x)
    fitness = 10.0 * len(array) + sum
    return fitness

def schwefel(array):
    sum = 0
    fitness = 0
    for x in array:
        sum = sum + x * np.sin(np.sqrt(np.abs(x)))
    fitness = 418.9829 * len(array) - sum
    return fitness

def michalewicz(array):#for the number of Dimension is 2
    sum = 0
    fitness = 0
    m = 10
    for (i,x) in enumerate(array, start=1):
        sum = sum + np.sin(x) * np.sin((i * (x**2) )/np.pi)**(2*m)
    fitness = -sum
    return fitness

if __name__ == '__main__':
    a = np.array([2.20,1.0])
    print (michalewicz(a))
