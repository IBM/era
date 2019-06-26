# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 16:58:08 2019

@author: Wooram Lee
"""

import numpy as np

def heaviside(t): return 0.5*(np.sign(t)+1)                 # Heaviside step function

def rectpuls(tm,t): return heaviside(t-0.01*tm)-heaviside(t-1.01*tm)  # Rectangle function (with modulation window)

def nextpow2(i):        # Returns the next power of of 2 greater than the input
    n=1.
    while n<i: n*=2.
    return int(n)

def hamming(L):         # Hamming window (symmetric)
    N=L-1
    n=np.arange(L)
    w=0.54-0.46*np.cos(2*np.pi*n/N)
    return w    

def fmcw_phase(alpha,tm,t):
    Nt=len(t)
    t0=t[0:int(Nt/2)]
    t1=t[int(Nt/2):Nt]
    phase0=0.5*alpha*t0**2
    phase1=0.5*alpha*t1**2
    #phase1=-0.5*alpha*t1**2-alpha*0.25*tm**2+alpha*tm*t1
    return np.append(phase0,phase1) 

def TX_sig(f0,alpha,duty_cycle,T,Ae,fe,t): 
    tm=T*duty_cycle
    Nt=len(t)
    Nd=int(Nt*duty_cycle)
    TX_sig0=np.zeros(Nt-Nd)
    t1=t[0:Nd]
    TX_sig1=(1-(10**(Ae/20)-1)*np.sin(2*np.pi*fe*t1/tm))*np.exp(2j*np.pi*(f0*t1+fmcw_phase(alpha,tm,t1)))
    return np.append(TX_sig0,TX_sig1) 
