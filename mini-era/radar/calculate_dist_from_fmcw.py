#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 21 13:36:08 2019

@author: ARUNPAIDIMARRI
"""

import argparse
import numpy as np
from scipy.fftpack import fft

def nextpow2(i):        # Returns the next power of of 2 greater than the input
    n=1.
    while n<i: n*=2.
    return int(n)

# %%
def calculate_distance_from_fmcw(data, fs, alpha, threshold=-100):
    N=len(data)
    c=3e8
    
    fft_data=1/N*fft(data)
    
    max_index = np.abs(fft_data).argmax()
    max_psd = np.abs(fft_data[max_index])**2/100
    #print('Maximum index:', max_index)
    #print('Maximum psd:', max_psd)
    if 10*np.log10(max_psd) > threshold:
        return (max_index*float(fs/N))*0.5*c/alpha
    else:
        return np.inf

# %%
def calculate_distances_from_fmcw(data, fs, alpha, threshold=-100):
    N=int(len(data))
    c=3e8
    
    fft_data=1/N*fft(data)
    psd_data=10*np.log10(np.abs(fft_data)**2/100)
    
    indices = []
    distances = []
    psds_peaks = []
    psds_troughs = []
    rising = True
    trough_prev = psd_data[0]
    for i in range(1,int(N/2)):
        if psd_data[i]>=psd_data[i-1] and not rising:
            rising=True
            trough_prev=psd_data[i-1]
            if len(indices)>0:
                psds_troughs[-1] = np.max([psds_troughs[-1], trough_prev])
        elif psd_data[i]<psd_data[i-1] and rising:
            rising=False
            indices.append(i-1)
            distances.append(((i-1)*fs/N)*0.5*c/alpha)
            psds_peaks.append(psd_data[i-1])
            psds_troughs.append(trough_prev)
    if len(indices)==0:
        indices.append(int(N/2)-1)
        distances.append(((int(N/2)-1)*fs/N)*0.5*c/alpha)
        psds_peaks.append(psd_data[int(N/2)-1])
        psds_troughs.append(psd_data[0])
    else:
        psds_troughs[-1] = np.max([psds_troughs[-1], psd_data[int(N/2)-1]])
    
    indices2 = []
    distances2 = []
    psds_peaks2 = []
    psds_troughs2 = []
    for i in range(len(indices)):
        if psds_peaks[i] > threshold:
            indices2.append(indices[i])
            distances2.append(distances[i])
            psds_peaks2.append(psds_peaks[i])
            psds_troughs2.append(psds_troughs[i])
    
    if len(indices2) == 0:
        indices2 = [-1]
        distances2 = [np.inf]
        psds_peaks2 = [-110]
        psds_troughs2 = [-200]
        
#    print('Output from parsing of PSD Data')
#    print('Indices of Peaks:  ', indices2)
#    print('Distances of Peaks:', distances2)
#    print('PSD if the Peaks:  ', psds_peaks2)
#    print('PSD of troughs:    ', psds_troughs2)
    
    return distances2

# %%
def argparser():
    parser = argparse.ArgumentParser(description = 'Calculate distance from FMCW data')
    parser.add_argument('file', metavar='FILE', help='Input file name')
    parser.add_argument('-m', '--max_only', dest='max_only', action='store_true',
                        help='Give out only the distance to the strongest (closest) object')
    return parser

# %%
if __name__ == '__main__':
    parser = argparser()
    args = parser.parse_args()
   
    '''radar setting condition'''
    B=2.4e9                 # Chirp bandwidth (2.4 GHz)
    T=500.e-6               # Chirp period (400 uS)
    duty_cycle=1          # Duty cycle   
    Td=T*duty_cycle         # Chirp duration 
    alpha=B/Td              # Chirp rate (saw-tooth) 
    # %
    '''ADC spec.'''
    fs=30.e6                   # Approximate Sampling frequency (30 MHz)
    Ts=1./fs                   # Sampling period (8 ns)
    N=nextpow2(int(T/Ts)) # Number of samples per sweep period (50,000)
    Ts = T/N
    fs=int(1./Ts)

    data = np.loadtxt(args.file, delimiter=',').view(np.complex)
    if args.max_only:
        dist = calculate_distance_from_fmcw(data, fs, alpha)
        print ('Distance of object from FMCW data = %f m' %(dist))
    else:
        dists = calculate_distances_from_fmcw(data, fs, alpha)
        print ('Distances of objects from FMCW data: ', dists)
