#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 16:58:08 2019

@author: ARUNPAIDIMARRI
"""
import argparse
import numpy as np
from scipy.fftpack import fft
import matplotlib.pyplot as plt
import fmcw_functions as fmcw

# %%
def calculate_fmcw_waveform(obj_dists=[100]):
    '''radar setting condition'''
    fc=10.e9                # IF Center frequency(10 GHz)
    B=2.4e9                 # Chirp bandwidth (2.4 GHz)
    f0=fc-B/2               # Chirp start frequency (10-2.4/2=8.8 GHz)
    T=500.e-6               # Chirp period (400 uS)
    duty_cycle=1          # Duty cycle   
    Td=T*duty_cycle         # Chirp duration 
    alpha=B/Td              # Chirp rate (saw-tooth) 
    
    ''' FMCW gain profile'''
    Ae=0.75*0               #Gain fluctuation (dB) +-Ae (dB) variation  
    fe=1                  #Number of fluctuation
    
    # %
    '''Target condition'''
    Rvals=obj_dists                   # Target range (1 m)
    c=3.e8                  # Speed of light (in m/s)
    tau_ext=0.e-9           # 0.5 ns extra delay through cables and IC. (needs to estimate)
    taus=[2.*R/c+tau_ext for R in Rvals]      # Two-way transit time (6.6 ns)
    amplitudes = [(10/R)**2 for R in Rvals]
    
    # %
    '''ADC spec.'''
    fs=30.e6                   # Approximate Sampling frequency (30 MHz)
    Ts=1./fs                   # Sampling period (8 ns)
    N=fmcw.nextpow2(int(T/Ts)) # Number of samples per sweep period (50,000)
    Ts = T/N
    fs=int(1./Ts)
    cutoff=(fs/2)*0.8     #low-pass filter cut-off frequency (low-pass)1MHz rad/s after beating
    
    # %
    ''' Time grid '''
    n=np.arange(N)
    t=n*Ts
    
    # %
    max_range = (fs/2)*(1/alpha)*(c/2)
        
    # %
    print ('Sample number= %3d' % (N))
    print ('Sampling frequency= %3f MHz' % (fs/1e6))
    print ('Chirp modulation time= %f us' % (Td*1e6))
    print ('Chirp total time (with zeroing)= %f us' % (T*1e6))
    print ('FFT frequency resolution :%f KHz' % (1/T/1e3))
    print ('Range resolution= %f m' % ((1/T)*(1/alpha)*(c/2)))
    print ('Maximum range of FMCW Radar= %f m\n\n' % (max_range*0.95))
    
    print ('Distance of objects for the FMCW data is=', Rvals)
    for i in range(len(Rvals)):
        if (Rvals[i] > max_range*0.95) or (Rvals[i] < 0):
            amplitudes[i]=1e-6
            
    # %
    #Calculation of beating signal
    TX_signal = fmcw.TX_sig(f0,alpha,duty_cycle,Td,Ae,fe,t)
    RX_signal = amplitudes[0]*fmcw.TX_sig(f0,alpha,duty_cycle,Td,Ae,fe,t-taus[0])
    for i in range(1, len(Rvals)):
        RX_signal += amplitudes[i]*fmcw.TX_sig(f0,alpha,duty_cycle,Td,Ae,fe,t-taus[i])
    beat_signal=TX_signal*np.conj(RX_signal)
    
    #Calculation of window function array
    window = np.append(np.zeros(N-int(duty_cycle*N)),np.hanning(int(N*duty_cycle))) #apply Hanning window
    beat_signal_window=window*beat_signal  
    
    '''Filtered output after low-pass filter with the cut-off frequency of fs'''
    #beat_signal=fmcw.TX_sig_filt(f0,alpha,0.9*T,f_env,fs,t)*np.conj(fmcw.TX_sig_filt(f0,alpha,0.9*T,f_env,fs,t-tau))
    #beat_filt=fmcw.butter_lowpass_filter(beat_signal,cutoff,fs,3)
    #beat_window_filt=fmcw.butter_lowpass_filter(beat_signal_window,cutoff,fs,3)
    return beat_signal_window, fs, alpha

# %%
def plot_fft(data, fs, alpha, threshold=-100, plot_time_domain=False):
    N = len(data)
    c=3e8
    
    #Original signal
    fft_data=1/N*fft(data)
    freq_step = fs/N
    freqs_data=np.arange(N)*freq_step
    psd_data=np.abs(fft_data)**2/100
    
    if plot_time_domain:
        fig = plt.figure(8, figsize = (6,4))
        fig.clf()
        ax = fig.add_subplot(1,1,1)
        ax.plot(np.arange(N)*1e6/fs, np.real(data))
        ax.set_xlabel('Time(us)', fontsize=15)
        fig.tight_layout()
    
    fig = plt.figure(9, figsize = (6,4))
    fig.clf()
    ax = fig.add_subplot(1,1,1)
    ax.plot(freqs_data*(0.5*c/alpha), 10 * np.log10(psd_data))
    ax.set_xlabel('Distance(m)', fontsize=15)
    ax.set_ylabel('PSD (dB)', fontsize=15)
    ax.set_ylim(threshold,20)
    ax.set_xlim(0.5,(0.25*c*fs/alpha))
    ax.set_xscale('linear')
    ax.grid()
    plt.tight_layout()
    plt.pause(0.5)
    return fig

# %%
def argparser():
    parser = argparse.ArgumentParser(description = 'Generate data for FMCW Radar', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--file', dest='file', default='temp.dat',
                        help='Output file name')
    parser.add_argument('-d', '--distances', dest='distances', nargs='+', type=float, default=[100],
                        help='Distance of object')
    parser.add_argument('-p', '--plot', dest='make_plot', action='store_true',
                        help='Plot FFT')
    return parser

# %%
if __name__ == '__main__':
    parser = argparser()
    args = parser.parse_args()
    data, fs, alpha = calculate_fmcw_waveform(args.distances)
    np.savetxt(args.file, data.view(float), delimiter=',')
    if args.make_plot:
        fig = plot_fft(data, fs, alpha)
        temp = input('Press enter to close plot:')
        plt.close(fig)
