#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 29 22:43:45 2021

@author: vinay
"""

import numpy as np 
from numpy import linalg
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt 
from math import pi as pi

import cmath

global math
math = np.matrix

global d1,a2,a3,d4,d5,d6

d1 = 0.089159
a2 = -0.425
a3 = -0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

global d,a,alpha

d= math([d1,0,0,d4,d5,d6])
a= math([0,a2,a3,0,0,0])
alpha = math([pi/2,0,0,pi/2,-pi/2,0])

def AH(n,th, c):
    T_a = math(np.identity(4),copy= False)
    T_a[0,3]= a[0,n-1]
    T_d = math(np.identity(4),copy= False)
    T_d[2,3]= d[0,n-1]
    
    
    Rzt = math([[cos(th[n-1,c]),-sin(th[n-1,c]),0,0],
               [sin(th[n-1,c]),cos(th[n-1,c]),0,0],
               [0,0,0,1],
               [0,0,0,1]],copy = False
               )
    Rxa = math([[1,0,0,0],
                [0,cos(alpha[0,n-1]),cos(alpha[0,n-1]),0],
                [ 0,sin(alpha[0,n-1]),cos(alpha[0,n-1]),0],
                [0,0,0,1],
                ],copy = False)
    
    A_i = T_d*Rzt*T_a*Rxa
    return A_i
    

def RTrans(th,c):

    A_1 =AH(1,th,c)    
    A_2= AH(2,th,c)    
    A_3= AH(3,th,c)    
    A_4 =AH(4,th,c)    
    A_5= AH(5, th,c)    
    A_6 =AH(6,th, c)    
    T_06 = A_1*A_2*A_3*A_4*A_5*A_6
    
    return T_06

def invKine(desired_pos):
    th = math(np.zeros((6,8)))      
    P_05 = (desired_pos *math([0,0,-d6,1]).T-math([0,0,0,1]).T)       
    psi = atan2(P_05[2-1,0],P_05[1-1,0])    
    phi = acos(d4/sqrt(P_05[2-1,0]*P_05[2-1,0]+P_05[1-1,0]*P_05[1-1,0]))
    
    #theta shoulder left or right
    
    th[0,0:4] = pi/2 + psi + phi    
    th[0,4:8] = pi/2 + psi -phi
    th = th.real

    cl = [0,4]#wrist is up or down
    for i in range(0, len(cl)):    
        c = cl[i]        
        T_10 = linalg.inv(AH(i,th,c))        
        T_16 = T_10*desired_pos        
        th[4,c:c+2] = + acos((T_16[2,3]-d4)/d6);
        th[4,c+2:c+4]= - acos((T_16[2,3]-d4)/d6);
        
    th = th.real    
    
    
    cl = [0,2,4,6]
    for i in range(0, len(cl)):    
        c = cl[i]        
        T_10 = linalg.inv(AH(i,th,c))        
        T_16 = linalg.inv(T_10*desired_pos)
        th[5,c:c+2] = atan2((-T_16[1,2]/sin(th[4,c])), (T_16[0,2]/sin(th[4,c])))
        
        
    cl = [0,2,4,6]
    for i in range(0,len(cl)):    
        c = cl[i]        
        T_10=  linalg.inv(AH(1,th,c))        
        T_65 = AH(6,th,c)        
        T_54 =AH(5,th,c)        
        T_14 = (T_10*desired_pos)*linalg.inv(T_54*T_65) 
        P_13 = T_14*math([0,-d4,0,1]).T-math([0,0,0,1]).T        
        t3 = cmath.acos((linalg.norm(P_13)**2-a2**2-a3**2)/(2*32*a3))        
        th[2,c] = t3.real        
        th[2,c+1] = -t3.real    
    th = th.real



    cl = [0,1,2,3,4,5,6,7]

    for i in range(0, len(cl)):    
        c = cl[i]        
        T_10 = linalg.inv(AH(1,th,c)) 
        T_65= linalg. inv(AH(6, th,c))        
        T_54 = linalg. inv(AH(5,th,c))        
        T_14 = (T_10*desired_pos)*T_65*T_54        
        P_13=T_14*math([0,-d4,0,1]).T -math([0,0,0,1]).T        
        #theta 2
        
        th[1,c] = -atan2 (P_13[1], -P_13[0])+asin(a3* sin(th[2,c])/linalg.norm(P_13))
        
        #theta 4

        T_32= linalg. inv(AH(3, th, c))        
        T_21= linalg.inv(AH(2,th, c))        
        T_34 = T_32*T_21*T_14        
        th[3,c] = atan2 (T_34[1,0],T_34[0,0])


    th = th.real
    return th

desired_pos= np.matrix([[ 3.06161700e-17, 8.66025404e-01, -5.0000000e-01, 3.63537488e-01],
                        [-1.00000000e+00, 0.00000000e+00, -5.55111512e-17, -1.09150000e-01], 
                        [-5.55111512e-17, 5.00000000e-01, 8.66025404e-01, 4.25598256e-01],
                        [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])




th = invKine(desired_pos)
print(th)




























