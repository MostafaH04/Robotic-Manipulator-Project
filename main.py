from math import sin, cos, pi, acos, atan
from Link import Link, GRAVITY

import matplotlib.pyplot as plt

LOAD_FORCE = 5 * GRAVITY

L1_CONST = 4
L2_CONST = 2
L3_CONST = 1

def dec_range(start, stop, increment):
    while start < stop:
        yield start
        start += increment

def momentCalc(links: list):
    sumMoment = 0;

    # moment by a link is x coordinate of center of mass
    # multiplied by force applied

    for link in links:
        COM = link.COM
        linkMoment = COM.posX * COM.force

        sumMoment += linkMoment
    # moment of the 5 kg Load
    sumMoment += links[-1].endX * LOAD_FORCE
    
    return abs(sumMoment)

def getCost(A,B,len3):

    originals = [[0.75,0.1,-60],[0.5,0.5,0],[0.2,0.6,45]]
    stored = []
    colors = ['bo', 'ro', 'go']
    count = 0
    moments = []
    failed = False
    for vals in originals:
        l3Len = len3
        q3 = vals[2] * pi/180
        finalPos = [vals[0], vals[1]]

        l3Width = l3Len * cos(q3)
        l3Height = l3Len * sin(q3)

        l3Start = [finalPos[0]-l3Width, finalPos[1]-l3Height]
        xPrime = l3Start[0]
        yPrime = l3Start[1]

        C = (xPrime**2 + yPrime **2) ** (1/2)

        # Changing parameters

        try:
            l1Len = A
            l2Len = B

            alpha = acos((A**2 - B**2 - C**2)/(-2 * B * C))
            theta = acos((C**2 - A**2 - B**2)/(-2 * A * B))
            beta = pi - alpha - theta

            q1 = beta + atan(yPrime/xPrime)
            q2 = atan(yPrime/xPrime) - alpha

            q1 *= 180/pi
            q2 *= 180/pi
            q3 *= 180/pi
            print(q1,q2,q3)
            l1 = Link([0,0], l1Len, q1, L1_CONST)
            l2 = Link(l1.end, l2Len, q2, L2_CONST)
            l3 = Link(l2.end, l3Len, q3, L3_CONST)

            links = [l1,l2,l3]

            x_values = [l1.startX, l1.endX, l2.startX, l2.endX, l3.startX, l3.endX]
            y_values = [l1.startY, l1.endY, l2.startY, l2.endY, l3.startY, l3.endY]

            plt.plot(x_values, y_values, colors[len(moments)], linestyle="--")
            for l in range(len(links)):
                plt.text(links[l].endX-0.0015, links[l].endY+0.025, l+1)

            if l1.endY < 0 or l2.endY < 0 or l3.endY < 0:
                failed = True
                continue

            # print("Successful Parameters")
            # print(l3.end)
            # print(q1,q2,q3)
            # print(momentCalc(links))
            moments.append(momentCalc(links))
        except:
            failed = True

    if not failed:
        momentT = 0
        for moment in moments:
            momentT += moment**2
        momentT = momentT**(1/2)
        stored.append({'Lengths': [l1Len,l2Len,l3Len],
                       'moments': moments,
                       'MomentT': momentT})
        print(stored)
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        plt.show()
        return momentT

    return 99999999999999

print(getCost(0.316,0.509,0.266))
#input()

'''
import numpy as np
store = []
X = []
Y = []
Z = []
for C in dec_range(0.02,0.03, 0.01):
    for A in dec_range(0,1.5,0.01):
        for B in dec_range(0,1.5,0.01):
            
            current = getCost(A,B,C)
            X.append(A)
            Y.append(B)
            Z.append(current)
            if current <= 0:
                continue
            print(A,B,C)
            print(current)

            store.append([current,[A,B,C]])
try:
    print(sorted(store,key = lambda x:x[0])[0:10])
except:
    pass

import scipy.optimize as optimize
from random import random
def optimizeArm():
    def objective(x):
        return getCost(x[0], x[1], x[2])

    initial_guess = [random(),random(),random()]
    
    bounds = [(0.01, 5.0), (0.01, 5.0), (0.01, 5.0)]

    result = optimize.minimize(objective, initial_guess, method='CG')

    if result.success:
        print(f"Optimization successful! Lengths: {result.x}")
    else:
        print("Optimization failed.")
    
    return result.x, objective(result.x)

store = [] 
for i in range(100):
    store.append(optimizeArm())

for i in store:
    print(i)
    '''