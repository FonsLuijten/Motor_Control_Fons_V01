#%%Definitions
"""
Created on Sat Sep 29 19:25:07 2018

@author: Elwin + Fons
"""

from os import chdir
#chdir('c:\\users\\elwin\\desktop\\python')
#chdir('D:\\OneDrive\\Projecten\\Pendulum\\Software\\V02 - Fons - Elwin\\Python')
chdir('C:\\Users\\fonsl\\OneDrive\\Projecten\\Mavilor motors\\Software\\V04 - Fons - Elwin\\Python')

import serial
import numpy as np
from scipy import signal as sig
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import struct
import warnings
import matplotlib.cbook
import time
import make3
import math
import pandas as pd
from tqdm import tqdm
import re

#f_pwm = 20e3
Ts = 50e-6 #1/(2*f_pwm)
n_trace = 10;
com = 'COM4'

from numpy import (atleast_1d, poly, polyval, roots, real, asarray,
                   resize, pi, absolute, logspace, r_, sqrt, tan, log10,
                   arctan, arcsinh, sin, cos, exp, cosh, arccosh, ceil, conjugate,
                   zeros, sinh, append, concatenate, prod, ones, array,
                   mintypecode)
from numpy.polynomial.polynomial import polyval as npp_polyval
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)


def tracesignal( signals, t ):
    setTrace( signals )
    t,s = readData( int(t /Ts) )
    plt.figure();
    plt.plot(t,s[:,0:len(signals)], );
    plt.legend( signals )
    plt.show()
    for signal in signals:
        varname = signal.replace("[", "_");
        varname = varname.replace("]", "");
        exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
    return t,s[:,0:len(signals)]

def bode( H ,f  , name = '' , title = 'bode plot'):
    mag = np.abs( H )
    phase = np.angle( H )
    ax1 = plt.subplot(2,1,1)
#    plt.loglog(f, mag)    # Bode magnitude plot
    plt.semilogx(f, 20*np.log10(mag) , label = name)    # Bode magnitude plot
    plt.grid( 1 , 'both' , 'both')
    plt.ylabel('Magnitude [dB]')
    plt.title(title)
    plt.legend()
#    ax1.xaxis.set_minor_formatter(ticker.ScalarFormatter())
    ax2 = plt.subplot(2,1,2, sharex=ax1)
    plt.plot(f, phase * 180 / np.pi)  # Bode phase plot
    plt.grid( 1 , 'both')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Phase [deg]')
    plt.ylim( -182, 182)
    plt.tight_layout()
    plt.show()   

def nyquist( H ,f  , name = '' , title = 'nyquist plot'):
    plt.plot( np.real( H ) , np.imag( H ) , label = name)    # Bode magnitude plot
    tmp = 0.5 * np.exp( -1j * np.arange( 0 , 2*np.pi , 0.01) ) - 1
    plt.plot( np.real( tmp ) , np.imag( tmp ) , label = '6 dB')    # Bode magnitude plot
    plt.grid( 1 , 'both')
    plt.title(title)
    plt.legend()
    plt.xlabel('Real')
    plt.ylabel('Imaginary')
    plt.xlim( -2 , 2)
    plt.ylim( -2 , 2)
    plt.tight_layout()
    plt.show()

def restart():
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

def setcont( contnum ):
    ser.write( b'C' + struct.pack('I',  contnum ) )
    
def moverel( pos ):
    p = pos
    v = 30*np.pi
    a = 300*np.pi
    j = 40000*np.pi
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )
    ser.write( b'p' + struct.pack('f',  0) )

def moveabs( pos ):
    posrel = pos - getsig('rmech')
    if posrel != 0:
        moverel( posrel )

def freqrespd( num , den ,f , Ts ):
    zm1 = exp(-1j * 2*np.pi * f*Ts )
    H = (npp_polyval(zm1, num, tensor=False) / npp_polyval(zm1, den, tensor=False))
    return H
   
def leadlag( fBW , alpha1, alpha2, Ts):
    fs = 1/Ts
    alpha = np.sqrt( 1/ alpha2**2 + 1 ) / np.sqrt( alpha1**2 + 1 )
    w0 = fBW * 2 * np.pi / alpha1
    wp = fBW * 2 * np.pi * alpha2
    a1 = (wp -2*fs) / (wp + 2*fs)
    b0 = alpha * ( ( 1 + 2*fs/w0 ) / ( 1 + 2*fs/wp ))
    b1 = alpha * ( ( 1 - 2*fs/w0 ) / ( 1 + 2*fs/wp ))
    num = [b0 , b1]
    den = [1 , a1]
    return num, den

def lowpass2( f0, damp,  Ts):
    w0 = 2 * np.pi * f0 * Ts
    alpha =sin(w0) * damp
    b0 = 0.5 * (1 - cos(w0)) / (1 + alpha) 
    b1 = 2 * b0
    b2 = b0 
    a1 = (-2 * cos(w0)) / (1 + alpha)
    a2 = (1 - alpha) / (1 + alpha)
    num = [b0 , b1, b2]
    den = [1  , a1, a2]
    return num, den

def Integrator( f0, Ts):
    b0 = f0 * 2 * pi * Ts/2
    b1 = b0
    num = [b0, b1]
    den = [1  , -1]
    return num, den

def fig():
    plt.figure()
    line1, = plt.plot( t , sens , label='sens')
    line1, = plt.plot( t , e , label='e')
    line1, = plt.plot( t , r , label='r')
    line1, = plt.plot( t , dist , label='dist')
    line1, = plt.plot( t , Vout , label='Vout')
    line1, = plt.plot( t , ss_f , label='ss_f')
    line1, = plt.plot( t , enc2 , label='enc2')
    line1, = plt.plot( t , emech , label='emech')
    plt.grid( 1 , 'both')
    plt.legend()
    plt.show()
    
#    tbegin = 1
#    tlength = 1
#    
#    jbegin = int( tbegin / Ts)
#    jend = jbegin + int( tlength / Ts)
#    
#    plt.figure()
#    line1, = plt.step( t[jbegin:jend] , sens[jbegin:jend] , label='sens')
#    line1, = plt.step( t[jbegin:jend] , e[jbegin:jend] , label='e')
#    line1, = plt.step( t[jbegin:jend] , r[jbegin:jend] , label='r')
#    line1, = plt.step( t[jbegin:jend] , dist[jbegin:jend] , label='dist')
#    line1, = plt.step( t[jbegin:jend] , Vout[jbegin:jend] , label='Vout')
#    line1, = plt.step( t[jbegin:jend] , ss_f[jbegin:jend] , label='ss_f')
#    line1, = plt.step( t[jbegin:jend] , enc2[jbegin:jend] , label='enc2')
#    plt.grid( 1 , 'both')
#    plt.legend()
#    plt.show()


def readData(N):
    Nsig = 2+n_trace
    N_total = 0
    # msgparts = []
    
    # if N>1:
    #     pbar = tqdm( desc = 'Reading data' , total = N *4*Nsig)
    setpar('Nsend' , N)
    # while N_total < N*4*Nsig:
    #     N_avail = ser.in_waiting
    #     if N_avail > 0:      
    #         chunk = ser.read(N_avail)
    #         msgparts.append(chunk)       # Add it to list of chunks
    #         N_total += N_avail            
    #         if N>1:
    #             pbar.update( N_avail )
    # buffer = b"".join(msgparts)          # Make the final message     
    if N < 100:
        buffer = ser.read(N*4*Nsig)
    else:
        buffer = ser.readall()
        
   # return buffer,buffer
 
    # if N>1:
    #     pbar.close()
 
    n = np.zeros( int(N) , dtype=np.uint32 )
    t = np.zeros( int(N) ) 
    s = np.zeros( ( int(N) , Nsig-2 ))
    
    sigtypes = ['I' , 'I' ] + ser.tracesigtypes
    for sigtype in set(sigtypes):
        indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
        indices = np.array(indices)
        if sigtype == 'b':
            tmp = np.ndarray( ( int(N) , Nsig ) , 'bool', buffer[0::4] )
        else:
            tmp = np.ndarray( ( int(N) , Nsig ) , sigtype, buffer )
        if sigtype == 'I':
            n = tmp[:,indices[0]]
            t = tmp[:,indices[1]] / 1e6
            s[:,indices[2:]-2] = tmp[:,indices[2:]]
        else:
            s[:,indices-2] = tmp[:,indices]
    
  #  sigtypes = ['I' , 'I' ] + ser.tracesigtypes
  #  for sigtype in set(sigtypes):
  #      indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
  #      indices = np.array(indices)
  #      tmp = np.ndarray( ( int(N) , Nsig ) , sigtype, buffer )
  #      if sigtype == 'I':
  #          n = tmp[:,indices[0]]
  #          t = tmp[:,indices[1]] / 1e6
  #          s[:,indices[2:]-2] = tmp[:,indices[2:]]
  #      else:
  #          s[:,indices-2] = tmp[:,indices]

    if N > 1:
        fit = np.polyfit( np.arange(0,N) , t , 1 )
        
        t -= t[0]
        Ndownsample = getsig('Ndownsample')
        if max( np.diff(n , axis=0) ) > Ndownsample:
            print('data seems BAD (diff n > 1)')
        elif (max(abs((np.diff( t) - Ts*Ndownsample))) < 5e-6) &  (np.abs(Ts*Ndownsample -fit[0]) < 5e-6):
            print('data seems OK')
        else:
            print('data seems BAD, Ts not OK')
            plt.figure(100)
            line1, = plt.plot( np.diff( t ,axis=0) *1e6 , label='Using set_dashes()')
            plt.grid( 1 , 'both')
            plt.show()
    return t,s

def trace( time ):
    t,s = readData( np.ceil(time/Ts).astype('int') )
    df = pd.DataFrame( s[:,0:len(ser.signalsnames)], columns=ser.signalsnames )
    df.index = t
    return df

def getFFT( signal , Ndownsample ):
    j0 = int(1/Ts) # These amount of samples are ignored
    L = 2047 * Ndownsample
    Naver = int( (len(signal)-j0)/L  )
#    print( 'Naver =' , Naver )
    SIGNAL = np.fft.fft( signal[j0:j0+L*Naver] , axis = 0)
    f  = np.fft.fftfreq(L*Naver, Ts)
    SIGNAL = SIGNAL[f>0]
    SIGNAL = 2*SIGNAL[Naver-1::Naver]   
    f = f[f>0]
    f = f[Naver-1::Naver]
    SIGNAL = SIGNAL[f< 1/(2*Ts*Ndownsample)]
    f = f[f< 1/(2*Ts*Ndownsample)]
    return SIGNAL, f

def getFFT_SS( signal ):
    SIGNAL = np.fft.fft( signal, axis = 0)
    f  = np.fft.fftfreq( len(signal) , Ts)
    SIGNAL = 2* SIGNAL[f>0] / len(signal)
    f = f[f>0]
    return SIGNAL, f

def fft( signal , name = ''):
    L = len(signal)
    SIGNAL = np.fft.fft( signal , axis = 0) / L
    f  = np.fft.fftfreq(L, Ts)
    SIGNAL = SIGNAL[f>0]
    f = f[f>0]
    bode( SIGNAL ,f  , name , title = 'FFT')
    return SIGNAL, f

def fftpsd( signal , name = ''):
    L = len(signal)
    SIGNAL = np.fft.fft( signal , axis = 0) / L
    f  = np.fft.fftfreq(L, Ts)
    SIGNAL = 2 * SIGNAL[f>0] 
    f = f[f>0]   

    ax1 = plt.subplot(2,1,1)
    plt.loglog( f , abs(SIGNAL)**2 /Ts )
    plt.grid( 1 , 'both' , 'both')
    plt.ylabel('PSD [-²/Hz]')
    # plt.title(title)
    plt.legend()
    ax2 = plt.subplot(2,1,2, sharex=ax1)
    plt.semilogx( f , np.cumsum(abs(SIGNAL)**2 /Ts) *Ts )
    plt.grid( 1 , 'both')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('CPS [-²]')
    plt.tight_layout()
    plt.show()   

    return SIGNAL, f


def analyseSS( freq ):
    I = ss_f == freq
    plt.figure();
    plt.plot( t2[I]-t2[I][0] , dist[I] , label='disturbance')
    plt.plot( t2[I]-t2[I][0] , r[I] , label='current setpoint')
    plt.plot( t2[I]-t2[I][0] , Vout[I] , label='Vout')
    plt.plot( t2[I]-t2[I][0] , sens[I] , label='current sensor')    
    plt.plot( t2[I]-t2[I][0] , emech[I] , label='error')
    plt.grid( 1 , 'both')
    plt.legend()
    plt.show()

    plt.figure();
    fft( dist[I] , 'disturbance');
    fft( r[I] , 'current setpoint') ;
    fft( Vout[I] , 'Vout') ;
    fft( sens[I] , 'cursens' );
    fft( emech[I] , 'error' );

def makebodesOL( Ndownsample = 1 , fignum = None):
    DIST,f = getFFT( dist , Ndownsample )
    VOUT,f = getFFT( Vout , Ndownsample )
    SENS,f = getFFT( sens , Ndownsample )
    E,f = getFFT( e , Ndownsample )
    #ENC2,f = getFFT( enc2 , Ndownsample )
    R,f = getFFT( r , Ndownsample )
    #EMECH,f = getFFT( emech , Ndownsample )

    S = E / DIST
    CL = SENS / DIST
    P = SENS / VOUT
    OL = 1/S - 1
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    bode( CL , f , 'Closed loop')
      
#    plt.figure()
#    nyquist( OL , f , 'Open loop')
    
#    Relec = 1/0.45
#    Lelec = 1/((5e3*2*np.pi*0.07))
#    F = (Lelec * 1j * 2*np.pi*f + Relec)
#    delay = np.exp( 1.7 * Ts * -f * 2 * np.pi * 1j )
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    bode( P , f , 'Plant')
#    bode( 1/F , f , 'Plant fit')
#    bode( 1/F * delay , f , 'Plant fit, delayed')
    
#    plt.figure()
#    bode( OL / (1 + OL), f , 'closed loop, no FF')
#    bode( (OL + P*Relec )/ (1 + OL), f , 'closed loop, with R FF')
#    bode( (OL + P*F )/ (1 + OL) , f , 'closed loop, with R and L FF')
#    bode( (OL*delay + P*F )/ (1 + OL) , f , 'closed loop, with R and L FF, delay compensated')
    
    Kp = 15
    fInt = 700
    num, den = Integrator( fInt , Ts)
    INTEGRATOR = freqrespd( num , den ,f , Ts )
    
#    Use these if disturbance was input at voltage without a controller:
#    plt.figure()
#    bode( CL * Kp * (INTEGRATOR+1) , f )
#    plt.figure()
#    nyquist( CL * Kp * (INTEGRATOR+1) , f )
    
    
#    num, den = lowpass2( fLP , 0.7 , Ts)
#    LP2 = freqrespd( num , den ,f , Ts )

    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    bode( OL , f , 'Open loop')
#    bode( P * Kp * (INTEGRATOR+1) , f , 'Open loop reconstructed')
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    nyquist( OL , f , 'Open loop')
#    nyquist( P * Kp * (INTEGRATOR+1) , f , 'Open loop reconstructed')

#    plt.figure()
#    bode( 1/(P * Kp * (INTEGRATOR+1) +1) , f , 'Sensitivity reconstructed')
#    bode( S, f , 'Sensitivity')

def makebodesCL( Ndownsample = 1 , fignum = None):
    DIST,f = getFFT( dist , Ndownsample )
    VOUT,f = getFFT( Vout , Ndownsample )
    SENS,f = getFFT( sens , Ndownsample )
    E,f = getFFT( e , Ndownsample )
    ENC2,f = getFFT( enc2 , Ndownsample )
    R,f = getFFT( r , Ndownsample )
    EMECH,f = getFFT( emech , Ndownsample )

    S = R / DIST
    OL = 1/S - 1
    
    Kt = 0.068 # Nm/A
#    enc2rad = -2*np.pi/2048
    PS = -EMECH / DIST/ Kt

    Pmech = PS / S

    Jmeas = 1/(abs( Pmech[20] ) * (2*np.pi*f[20])**2  )
    Jmot = 0.0000325
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
    bode( Pmech , f , 'Pmech [rad/Nm]')
    
    #plt.figure()
    #bode( np.exp( -f * 2 * np.pi * Ts * 1j ) , f , 'time delay')
    
#    Kp = 4
#    fBW = 50
#    alpha1 = 3
#    alpha2 = 4
#    fInt = fBW / 6
#    fLP = fBW * 6
    Kp = 15
    fBW = 100
    alpha1 = 3
    alpha2 = 4
    fInt = fBW / 6
    fLP = fBW * 8
    
    num, den = leadlag( fBW , alpha1, alpha2, Ts)
    CONT = freqrespd( num , den ,f , Ts )
    num, den = lowpass2( fLP , 0.7 , Ts)
    LP2 = freqrespd( num , den ,f , Ts )
    num, den = Integrator( fInt , Ts)
    INTEGRATOR = freqrespd( num , den ,f , Ts )

#    plt.figure()
#    bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')


#    plt.figure()
#    bode( CONT * Kp , f , 'lead-lag')
#    bode( LP2 , f , 'LP')
#    bode( (INTEGRATOR+1) , f , 'INTEGRATOR')
#    bode( CONT * LP2 * Kp * (INTEGRATOR+1) , f , 'total')
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
#    bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')
    bode( OL , f , 'Open loop')

    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
    nyquist( OL , f , 'Open loop')

    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
    bode( R / DIST  , f , 'S')

#    Kp = 37.6
#    fBW = 150
#    alpha1 = 3
#    alpha2 = 4
#    fInt = fBW / 6
#    fLP = fBW * 6
#    
#    num, den = leadlag( fBW , alpha1, alpha2, Ts)
#    CONT = freqrespd( num , den ,f , Ts )
#    num, den = lowpass2( fLP , 0.7 , Ts)
#    LP2 = freqrespd( num , den ,f , Ts )
#    num, den = Integrator( fInt , Ts)
#    INTEGRATOR = freqrespd( num , den ,f , Ts )
#
#    plt.figure()
#    bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL Tuned')
#    
#    plt.figure()
#    bode( 1/(1+CONT * LP2 * Pmech * Kp* (INTEGRATOR+1))  , f , 'S Tuned')
"""
def start():
    ser = serial.Serial('COM3' , timeout= 2 )  # open serial port
    print(ser.name)         # check which port was really used
    #ser.write( b'K' + struct.pack('f',  0) )
    
    vFF = 0.0003
    Jload = 0.000041 
    ser.write( b'J' + struct.pack('f',  Jload ) )
    ser.write( b'V' + struct.pack('f',  vFF) )
    ser.write( b'R' + struct.pack('f',  0.0) )
    return ser
"""

try:
    ser = serial.Serial( com  , timeout= 0.1 )  # open serial port 0.1
    print(ser.name)         # check which port was really used
except:
    print(ser.name)         # check which port was really used

def start( ser ):
    try:
        ser.close();
        # ser.set_buffer_size(rx_size = 128000, tx_size = 128000)
        ser.open();
        time.sleep(2) #Wait till teensy is booted
    except:
        print('Failed to open serial port')    
    ser.signames, ser.sigtypes, ser.sigsizes = getsignalnames( ser );  # This generates overloads, TODO fix  
    ser.tracesigtypes= n_trace*['f']
    ser.signals= n_trace*[0]
    vFF = 0.0003
    Jload = 0.000041 
    setpar( 'Jload' ,  Jload )
    setpar( 'velFF' , vFF )
    setpar('R' , 0 )
    return ser

def setTrace( signals ):
    if isinstance( signals , str) or isinstance( signals , int):
        signals = [ signals ];
      
    ser.signalsnames = signals
    signals = []
    for signal in ser.signalsnames:
        if isinstance( signal , str):
            matches = signal if signal in ser.signames else []
            if matches:
                signals.append(matches)
            else:
                matches = [c for c in ser.signames if c.startswith(signal+"[")]
                if matches:
                    for c in matches:
                        signals.append(c)
                
    ser.signalsnames = signals
    i=0
    for signal in ser.signalsnames:
        if isinstance( signal , str):
            signal = ser.signames.index( signal )
        ser.signals[i] = signal
        ser.tracesigtypes[i] = ser.sigtypes[signal]
        ser.write( b't' + struct.pack('I', signal) + bytes(bytearray([i])) )
        i+=1
    return signals
    
    
def getsig( signals ):
    if isinstance( signals , str):
        signals = [ signals ];
    signals = setTrace( signals )
    t,s = readData( 1 )
    
    if len(signals) == 1:
        s = s[0,0];
    else:
        s = s[0,0:len(signals)];
#    print( s)
    return(s)


def setpar( signals , values ):
    if isinstance( signals , str) or isinstance( signals , int):
        signals = [ signals ];
    if isinstance( values , str) or isinstance( values , int) or isinstance( values , float):
        values = [ values ];
        
    ser.signalsnames = signals
    signals = []
    for signal in ser.signalsnames:
        if isinstance( signal , str):
            matches = signal if signal in ser.signames else []
            if matches:
                signals.append(matches)
            else:
                matches = [c for c in ser.signames if c.startswith(signal+"[")]
                if matches:
                    for c in matches:
                        signals.append(c)
    
    if len(signals) != len(values):
        warnings.warn("WARNING: the values do not match the number of variables")
    else:
        length = len(signals)
        for ii in range(length):
            signal = ser.signames.index( signals[ii] )
            value = values[ii]
            ser.write( b'S' + struct.pack('I',  signal) + struct.pack( ser.sigtypes[signal] ,  value)  )


def getsignalnames( ser ):
    ser.write( b'T' + struct.pack('I', 0));
    a = ser.readlines()
    signames = []
    for signame in a[:10000:3]:
        signames.append(signame.strip().decode("utf-8"))
    sigtypes = []
    for signame in a[1:10000:3]:#a[1:-1:2]:
        sigtypes.append(signame.strip().decode("utf-8"))
    sigsizes = []
    for signame in a[2:10000:3]:#a[1:-1:2]:
        sigsizes.append(signame.strip().decode("utf-8"))
    return signames, sigtypes, sigsizes

#CLose the loop
def CL(ser):
    ser = start( ser )
    
    setpar('ridethewave' , 1)
    time.sleep( 2 )
    if getsig( 'IndexFound1' ) < 1:
        print('index not found')
    else:    
        Jload = 0.0010
        vFF = 0.06
        setpar( 'Jload' ,  Jload )
        setpar( 'velFF' ,  vFF )
        setpar( 'rmechoffset' , 0)
        setpar( 'rmechoffset' , -getsig( 'emech' ))
        getsig('emech' )
        
        setpar( 'Icontgain' , 2 ) # Current loop gain
        ser.write( b'9' + struct.pack('f',  500) ) # Current loop integrator frequency

        
        ser.write( b'C' + struct.pack('I',  1) ) # 100 Hz BW
#        ser.write( b'C' + struct.pack('I',  2) ) # Low BW
        #ser.write( b'C' + struct.pack('I',  3) ) # Very low BW
#        ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW
  
    
def prepSP(  p , v , a , j ):
    t1, t2, t3, jd = make3.make3( p , v , a , j , Ts );
    #double tstart, double t1, double t2, double t3, double p, double v_max, double a_max, double j_max
    ser.write( b'1' + struct.pack('f',  t1) )
    ser.write( b'2' + struct.pack('f',  t2) )
    ser.write( b'3' + struct.pack('f',  t3) )
    ser.write( b'4' + struct.pack('f',  p) )
    ser.write( b'5' + struct.pack('f',  v) )
    ser.write( b'6' + struct.pack('f',  a) )
    ser.write( b'7' + struct.pack('f',  jd) )
    return t1, t2, t3, jd
    
def res():
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

def PCon(setstate):
    ser.write( b'n' + struct.pack('f',  0) ) # Reset integrators
    setpar('EncPos1Raw',0)
    setpar('EncPos2Raw',0)
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
    setpar('PCstate',setstate)  
        
def PCoff():
    ser.write( b'n' + struct.pack('f',  0) ) # Reset integrators
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
    setpar('PCstate',0)    
    
def normalize(v):
    norm = max(abs(v))
    if norm == 0: 
       return v
    return v / norm

# If you lose samples and/or timing of trace data is not OK it is likely due to 
# the PC not being fast enough to empty the buffers. e.g. running youtube in the 
# background on my laptop causes this problem. 

ser = start( ser )

#%% States
# 0: PWM off
# 1: Set direct PWM
# 2: Homing
# 3: Commutation
# 4: Current control

#%% OK Start Serial communication
ser = start(ser)

#%% OK Stop Serial communication
ser.close()

#%% OK Close all figures
plt.close('all')

#%% OK Restart Controller
setpar( 'state' , 0)
setpar( 'PCstate' ,0)   
setpar( 'PWMph1offs' , [0,0,0])
setpar( 'muziek_gain' , 0) # music current
setpar( 'muziek_gain_V' , 0) # music current
setpar('offsetVel',0)
setpar('offsetVelTot',0)
setpar('errcode',0)
ser.write( b'n' + struct.pack('f',  0) ) # Reset integrators
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


#%% OK Get signal
signals = ['Vph1']
signals = ['PWMph1']
signals = ['errcode']
#signals = ['Im1cal']
#signals = ['Im1']
#signals = ['PWMph1offs']
#signals = ['OutputOn']
s = getsig( signals )
print(s)


#%% OK Read and plot data
signals = [ 'EncPos1Raw','EncPos2Raw','pmechy','pmechphi']
signals = [ 'pmechy','TET','timeremain']
signals = [ 'rmechy','pmechy','emechy']
signals = [ 'pmechphi','vmechphi','pmechy','vmechy']
#signals = [ 'A_sincos','B_sincos','pmechphi','TET']
#signals = [ 'timePrev']
#signals = [ 'PCstate']
#signals = [ 'emechphi']
#signals = [ 'Vout']
signals = [ 'EncPos1Raw','PWMph1']
#signals = [ 'EncPos1Raw']

signals = [ 'Im1']
#signals = ['Im1cal']
signals = [ 'penc1','ThetaE1']
#signals = [ 'EncPos1Raw']
# signals = ser.signames[0:9] + ['timeremain']
signals = setTrace( signals )
t,s = readData( int(5/Ts) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()


#v = np.gradient(pmechy, t[1]-t[0])
#plt.plot(t,v, );
#plt.show()

#%% OK direct setting PWM
PWM = [0.5,0.5,0.5]
setpar( 'state' , 1)
setpar( 'PWMph1offs' , PWM)

#%% OK Zeroing encoders through OL setting PWM 6 step
setpar( 'state' , 2) 

Vbus = 12;
Vhome = 2;
PWMoff = [[-1,1,1], [-1,-1,1], [1,-1,1], [1,-1,-1], [1,1,-1], [-1,1,-1]]
PWMoff = np.dot(PWMoff, 0.5*Vhome/Vbus)
PWMoff = np.add(PWMoff, 0.5)
#PWMoff = [[0.4,0.6,0.6], [0.4,0.4,0.6], [0.6,0.4,0.6], [0.6,0.4,0.4], [0.6,0.6,0.4], [0.4,0.6,0.4]]

i = 0
for ii in range(6*5):
    PWM = PWMoff[i];
    setpar( 'PWMph1offs' , PWM) # Kp = 0 
    time.sleep(0.05)
    i = i + 1;
    if i == 6:
        i = 0
setpar( 'PWMph1offs' , [0,0,0])
time.sleep(0.1)
setpar( 'state' , 0)

#%% Calibration of commutation angle


ss_fstart = 10;
ss_fstep = 20;
ss_fend = 9999;
ss_n_aver = 100;
ss_distgain = 0.2;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )

N = int( t_tot / Ts)

setpar( 'PCstate' , 0) # Kp = 0
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

time.sleep(0.5)

ser.write( b's' + struct.pack('f', ss_distgain ) )
signals = ['dist','e','Vout','sens','r','ss_f']
setTrace( signals )
t,s = readData( int(N) )
ser.write( b's' + struct.pack('f',  0) )

setpar( 'offsetVel' , 0.0*np.pi)
setpar( 'distoff' , 0)        # disturbance offset

for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
CL = np.zeros( len(freqs) ,dtype=np.complex_)
P = np.zeros( len(freqs) ,dtype=np.complex_)
S = np.zeros( len(freqs) ,dtype=np.complex_)
C = np.zeros( len(freqs) ,dtype=np.complex_)
PS = np.zeros( len(freqs) ,dtype=np.complex_)
PS2 = np.zeros( len(freqs) ,dtype=np.complex_)

Kt = 0.068 # Nm/A
for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')])
    R , f = getFFT_SS(  s[I, signals.index('r')] )
    SENS , f = getFFT_SS(  s[I, signals.index('sens')] )
    VOUT , f = getFFT_SS(  s[I, signals.index('Vout')] )
    E , f = getFFT_SS(  s[I, signals.index('e')] )
    index = np.argmin( abs(freq-f ))
    CL[freq==freqs] = SENS[index] / DIST[index]
    P[freq==freqs] = SENS[index] / VOUT[index]  
    S[freq==freqs] = E[index] / DIST[index]  
    C[freq==freqs] = VOUT[index] / E[index]  

plt.figure(1)
bode( CL , freqs , 'CL')
plt.figure(2)
bode( P , freqs , 'P')
plt.figure(3)
bode( 1/S-1 , freqs , 'OL')
plt.figure(4)
bode( C , freqs , 'C')

#plt.figure(5);
#plt.plot(t,sens)

#plt.figure(6);
#plt.plot(t,Vout)

#fft(sens,name='sens')

#%% OK PRBS closed Current loop FRF measurement 
Ndownsample = 1
N = 10*2047 + 1/Ts
setpar( 'PCstate' , 0)
setpar( 'Ndownsample' , Ndownsample) #Downsampling
setpar( 'distval' , 0.3) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
signals = ['dist','e','Vout','sens','r','ss_f']
setTrace( signals )
t,s = readData( int(N) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
setpar( 'distval' , 0) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar( 'Ndownsample' , 1) #Downsampling
makebodesOL( Ndownsample , 1)

#%% OK Music on current
#ser.write( b'E' + struct.pack('f',  12*np.pi) )
setpar( 'muziek_gain' , 0.7) # music current

# 0.7 for satisfaction

#%% OK Music off current
setpar( 'muziek_gain' , 0) # music current

#%% OK Music on voltage
setpar( 'muziek_gain_V' , 12) # music current

#%% OK Music off voltage
setpar( 'muziek_gain_V' , 0) # music current
setpar('offsetVel',0)

#%% OK Single sine open Position loop identification

ss_fstart = 5;
ss_fstep = 1;

ss_fend = 999;
ss_n_aver = 10;
ss_distgain = 0.4;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )

N = int( t_tot / Ts)
#ser.write( b'b' + struct.pack('I',  0) )  # close brake

setpar( 'PCstate' , 0) # Kp = 0
setpar( 'offsetVel' , 0.0*np.pi)
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

time.sleep(0.5)

ser.write( b's' + struct.pack('f', ss_distgain ) )
signals = [ 'ss_f' , 'dist' ,'pmechy' ,'pmechphi']
setTrace( signals )
t,s = readData( int(N) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
ser.write( b's' + struct.pack('f',  0) )

t2,s2 = tracesignal(['Kt'],Ts)
Kt = s2[:,0].item()
dist = dist*Kt

setpar( 'distoff' , 0)
setpar( 'offsetVel' , 0.0*np.pi)

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
P = np.zeros( len(freqs) ,dtype=np.complex_)
P2 = np.zeros( len(freqs) ,dtype=np.complex_)
for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')]*Kt )
    PMECHY , f = getFFT_SS( s[I, signals.index('pmechy')] )
    PMECHPHI , f = getFFT_SS( s[I, signals.index('pmechphi')] )

    index = np.argmin( abs(freq-f ))  

    P[freq==freqs] = PMECHY[index] / DIST[index]  
    P2[freq==freqs] = PMECHPHI[index] / DIST[index]   

plt.figure(1)
bode( P , freqs , 'P')
plt.figure(2)
bode( P2 , freqs , 'P')
#bode( PS/S , freqs , 'P')
#bode( PS2/S , freqs , 'P2')

freq = ss_f[ np.argmin( abs(10-ss_f ) ) ]
plt.figure();
plt.plot(normalize( s[ ss_f == freq , signals.index('dist')]*Kt ))
plt.plot(normalize( s[ ss_f == freq , signals.index('pmechy')] ))
plt.show();

#%% Home x and phi
setpar('EncPos1Raw',0)
setpar('EncPos2Raw',5000) # Downward position = -pi

#%% OK Turn on position control
PCstate = 3
if PCstate == 1:
    ser.write( b'C' + struct.pack('I',  1) ) # 50 Hz BW
    setpar('Kp_PC1',5000) 
elif PCstate == 2:
    setpar('EnStateFB',0) 
elif PCstate == 3:
    ser.write( b'C' + struct.pack('I',  3) ) # 50 Hz BW
    setpar('Kp_PC1',263) 
    setpar('Kp_PC2',-0.04)
    setpar('Kp_PC2',-0.15)
else:
    setpar('Kp_PC1',0) 
    setpar('Kp_PC2',0) 
    setpar('EnStateFB',0) 
    

PCon(PCstate)

#%% OK Turn off position control
setpar('Kp_PC1',0) 
setpar('Kp_PC2',0) 
PCoff()

#%% OK Single sine closed Position loop identification

ss_fstart = 5;
ss_fstep = 1;
ss_fend = 150;
ss_n_aver = 10;
ss_distgain = 0.5;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )

N = int( t_tot / Ts)
#ser.write( b'b' + struct.pack('I',  0) )  # close brake

#setpar( 'Kp' , 0) # Kp = 0
setpar( 'offsetVel' , 0.0*np.pi)
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

#setpar( 'IntegOn' , 0)

time.sleep(0.5)

ser.write( b's' + struct.pack('f', ss_distgain ) )
signals = [ 'ss_f' , 'dist' ,'cursp' ,'emechy' , 'emechphi']
setTrace( signals )
t,s = readData( int(N) )
#n,t,t2,dist,emechy,pmechy,emechphi,pmechphi,rcur,ss_f,Ts = readDataPC(N)
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
ser.write( b's' + struct.pack('f',  0) )

t2,s2 = tracesignal(['Kt'],Ts)
Kt = s2[:,0].item()
dist = dist*Kt

setpar( 'distoff' , 0)
setpar( 'offsetVel' , 0.0*np.pi)

#setpar( 'IntegOn' , 1)

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
S = np.zeros( len(freqs) ,dtype=np.complex_)
PS = np.zeros( len(freqs) ,dtype=np.complex_)
PS2 = np.zeros( len(freqs) ,dtype=np.complex_)
for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')]*Kt )
    CURSP , f = getFFT_SS( s[I, signals.index('cursp')]*Kt )
    EMECHY , f = getFFT_SS( s[I, signals.index('emechy')] )
    EMECHPHI , f = getFFT_SS( s[I, signals.index('emechphi')] )

    index = np.argmin( abs(freq-f ))  

    S[freq==freqs] = CURSP[index] / DIST[index]
    PS[freq==freqs] = -EMECHY[index] / DIST[index]  
    PS2[freq==freqs] = -EMECHPHI[index] / DIST[index]     

plt.close('all')
plt.figure(1)
bode( PS/S , freqs , 'P')
plt.figure(2)
bode( PS2/S , freqs , 'P2')

plt.figure(3)
bode( 1/S-1 , freqs , 'OL')
plt.figure(4)
nyquist( 1/S-1 , freqs , 'OL')

plt.figure(5)
bode( (1/S-1)/(PS/S) , freqs , 'C')

plt.figure(6)
bode( S , freqs , 'S')

freq = ss_f[ np.argmin( abs(10-ss_f ) ) ]
plt.figure();
plt.plot( normalize( s[ ss_f == freq , signals.index('dist')]*Kt ))
plt.plot( normalize( s[ ss_f == freq , signals.index('cursp')]*Kt ))
plt.plot(normalize( s[ ss_f == freq , signals.index('emechy')] ))
plt.show();

#%% Overall FRF measurement PRBS
Ndownsample = 1
N = 30*2047 + 1/Ts
ser.write( b'Z' + struct.pack('I',  2) ) # Send data 1
ser.write( b'E' + struct.pack('f',  12*np.pi) )
ser.write( b'D' + struct.pack('I',  Ndownsample) )
ser.write( b'A' + struct.pack('f',  2) )
n,t,t2,dist,e,Vout,sens,r,ss_f,enc1,enc2,rmechy,pmechy,emechy,rmechphi,pmechphi,emechphi,Ts = readData( int(N) )
ser.write( b'E' + struct.pack('f',  0) )
ser.write( b'A' + struct.pack('f',  0) )
ser.write( b'D' + struct.pack('I',  1) )
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
makebodesCL( Ndownsample , 1)

#%% Set 50Hz bandwidth
ser.write( b'C' + struct.pack('I',  1) ) # 50 Hz BW

#%% Set 100Hz bandwidth
ser.write( b'C' + struct.pack('I',  2) ) # 100 Hz BW

#%% Set 150Hz bandwidth
ser.write( b'C' + struct.pack('I',  3) ) # 150 Hz BW

#%% Set 200Hz bandwidth
ser.write( b'C' + struct.pack('I',  4) ) # 200 Hz BW

#%% Single sine
ser.write( b'Z' + struct.pack('I',  1) ) # Send data 1
ss_fstart = 100;
ss_fstep = 2;
ss_fend = 5000;
ss_n_aver = 10;
ss_distgain = 0.5;

t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )
N = int( t_tot / Ts)
ser.write( b'E' + struct.pack('f',  12*np.pi) )
ser.write( b'f' + struct.pack('f',  ss_fstart) )    # ss_fstart
ser.write( b'F' + struct.pack('f',  ss_fend) )     # ss_fend
ser.write( b'g' + struct.pack('f',  ss_fstep) )      # ss_fstep
ser.write( b'G' + struct.pack('I',  ss_n_aver) )    # ss_n_aver
time.sleep(0.5)
ser.write( b's' + struct.pack('f', ss_distgain ) )
n,t,t2,dist,e,Vout,sens,r,ss_f,enc1,enc2,rmechy,pmechy,emechy,rmechphi,pmechphi,emechphi,Ts = readData(N)
ser.write( b's' + struct.pack('f',  0) )
ser.write( b'E' + struct.pack('f',  0*np.pi) )
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

freqs = np.unique( ss_f[ss_f>0]  )
S = np.zeros( len(freqs) ,dtype=np.complex_)
PS = np.zeros( len(freqs) ,dtype=np.complex_)
Pelec = np.zeros( len(freqs) ,dtype=np.complex_)
Kt = 0.068 # Nm/A
for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( dist[I] )
    R , f = getFFT_SS( r[I] )
    EMECH , f = getFFT_SS( emech[I] )
    SENS , f = getFFT_SS( sens[I] )
    VOUT , f = getFFT_SS( Vout[I] )
    index = np.argmin( abs(freq-f ))
    Pelec[freq==freqs] = SENS[index] / VOUT[index]  
    S[freq==freqs] = R[index] / DIST[index]
    PS[freq==freqs] = -EMECH[index] / DIST[index] / Kt  
    
plt.figure(1)
bode( PS/S , freqs , 'P')
plt.figure(2)
bode( 1/S-1 , freqs , 'OL')
plt.figure(3)
nyquist( 1/S-1 , freqs , 'OL')
plt.figure(4)
bode( Pelec , freqs , 'Pelec' )

#%% Controller design

Kt = 0.068
P = PS / S / Kt
f = freqs

Kp = 60
fBW = 200
alpha1 = 3
alpha2 = 4.5
fInt = fBW / 6
fLP = fBW * 8

#Kp = 37
#fBW = 150
#alpha1 = 3
#alpha2 = 4.5
#fInt = fBW / 6
#fLP = fBW * 8

num, den = leadlag( fBW , alpha1, alpha2, Ts)
CONT = freqrespd( num , den ,f , Ts )
num, den = lowpass2( fLP , 0.7 , Ts)
LP2 = freqrespd( num , den ,f , Ts )
num, den = Integrator( fInt , Ts)
INTEGRATOR = freqrespd( num , den ,f , Ts )

plt.figure()
bode(  1/S-1 , f , 'OL')
bode( CONT * LP2 * P * Kp* (INTEGRATOR+1)  , f , 'OL tuned')

plt.figure()
nyquist(  1/S-1 , f , 'OL')
nyquist( CONT * LP2 * P * Kp* (INTEGRATOR+1)  , f , 'OL tuned')

#%% Save data
np.savez( 'testSS' ,  n=n ,t=t,t2=t2,dist=dist,e=e,Vout=Vout,sens=sens,r=r,ss_f=ss_f,enc2=enc2,emech=emech,Ts=Ts )
npzfile = np.load('testNoLP.npz')
npzfile = np.load('test4HzLP.npz')
npzfile.files

n = npzfile['n']
t = npzfile['t']
t2 = npzfile['t2']
dist = npzfile['dist']
e = npzfile['e']
Vout = npzfile['Vout']
sens = npzfile['sens']
r = npzfile['r']
ss_f = npzfile['ss_f']
enc2 = npzfile['enc2']
emech = npzfile['emech']
Ts = npzfile['Ts']
    


#%% Open brake
ser.write( b'b' + struct.pack('I',  1) )  # open brake

#%% Load and play setpoint

Jload = 0.0
vFF = 0.0

setpar( 'Jload' ,  Jload )
setpar( 'velFF' , vFF )
setpar( 'R' , 0.0 )


p = -0.15
v = 200
a = 200
j = 100
Ts = 50e-6

[t1, t2, t3, jd] = prepSP(  p , v , a , j )

signals = [ 'rmechy', 'pmechy' , 'emechy' ,'acc' ,'mechcontout']
setTrace( signals )

ser.write( b'p' + struct.pack('f',  0) )
N = int( (1.2*(0.01+4*t1+2*t2+t3)+0.1) / Ts ) 
t,s = readData( int(N) )
ser.write( b'p' + struct.pack('f',  0) )

#t -= t[np.where(abs(s[:,signals.index('acc')]>0))[0][0]-1];

plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()

plt.figure();
plt.plot( t , s[ : , signals.index('rmechy')], label='rmech' );
plt.plot( t , s[ : , signals.index('pmechy')], label='ymech' );
plt.plot( t , s[ : , signals.index('emechy')], label='emech' );
plt.plot( t , s[ : , signals.index('acc')]*Jload , label='acc' );
plt.plot( t , s[ : , signals.index('mechcontout')] , label='mechcontout' );
plt.legend( )
plt.grid( 1 , 'both')
plt.show()



#%% Plot setpoint
plt.figure()
line1, = plt.plot( t[0:-2]+Ts , np.diff(np.diff(rmech)) /Ts**2, label='ref')
line1, = plt.plot( t , acc  , label='acc')
line1, = plt.plot( t , emech *1000000 , label='error')
plt.grid( 1 , 'both')
plt.legend()
plt.show()

#%% GUI elwin

p = 10
v = 30*np.pi
a = 300*np.pi
j = 40000*np.pi

 

[t1, t2, t3, jd] = prepSP(  p , v , a , j )

 


signals = [ 'ymech', 'Iq_meas', 'Id_meas']
signals = [ 'ymech', 'Iq_meas', 'firsterror']
signals = [ 'encoderPos1', 'encoderPos2', 'acc' ,'Iq_meas', 'mechcontout']
# signals = [ 'tA', 'tB', 'tC']
setTrace( signals )

 

import time
import random
import pyqtgraph as pg
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore

 

from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QPushButton,
    QWidget,
)

 

 

y1 = deque()
y1a = deque()
y2 = deque()
y3 = deque()
y4 = deque()
maxLen = 100#max number of data points to show on graph
app = QtGui.QApplication([])
# win = pg.GraphicsWindow()

 


win = pg.GraphicsLayoutWidget()

 

layout = QtGui.QGridLayout()

 


increasebutton = QtGui.QPushButton("Restart controller")
decreasebutton = QtGui.QPushButton("Start Setpoint")
decreasebutton2 = QtGui.QPushButton("Start Setpoint2")
setspbutton = QtGui.QPushButton("Set Setpoint")
setpointdist1 = QtGui.QLineEdit("1")
setpointdist2 = QtGui.QLineEdit("100")
setpointdist3 = QtGui.QLineEdit("1000")
setpointdist4 = QtGui.QLineEdit("100000")

 

    
layout.addWidget(increasebutton, 0, 0)
layout.addWidget(decreasebutton, 1, 0)
layout.addWidget(decreasebutton2, 2, 0)
layout.addWidget(setspbutton, 3, 0)
layout.addWidget( setpointdist1 , 4,0)
layout.addWidget( setpointdist2 , 5,0)
layout.addWidget( setpointdist3 , 6,0)
layout.addWidget( setpointdist4 , 7,0)

 

def on_increasebutton_clicked():
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

 

def on_decreasebutton_clicked():
    setpar('SPdir' , 1)
    setpar('spGO' , 1)
    
def on_decreasebutton_clicked2():
    setpar('SPdir' , 0)
    setpar('spGO' , 1)

 

def on_setsp_clicked():
    p = float(setpointdist1.text())
    v = float(setpointdist2.text())
    a = float(setpointdist3.text())
    j = float(setpointdist4.text())
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )

 


increasebutton.clicked.connect(on_increasebutton_clicked)
decreasebutton.clicked.connect(on_decreasebutton_clicked)
decreasebutton2.clicked.connect(on_decreasebutton_clicked2)
setspbutton.clicked.connect(on_setsp_clicked)

 

 

plot1 = pg.plot()
plot2 = pg.plot()
plot3 = pg.plot()

 


layout.addWidget(plot1, 0, 1)
layout.addWidget(plot2, 1, 1)
layout.addWidget(plot3, 2, 1)

 

   
curve1 = plot1.plot()
curve1a = plot1.plot(pen=(1,3))
curve2 = plot2.plot()
curve3 = plot3.plot()
curve4 = plot3.plot(pen=(1,3))

 

T = deque()

 

win.resize( 1000, 700)
win.setLayout(layout)
win.show()

 

setpar('Ndownsample' , int( 100 ))
setpar('Nsend' , int(1e6))

 


Nget=10
Nsig = 2+n_trace
while not win.isHidden():
    while ~ser.in_waiting >= Nget*(2+n_trace)*4:
        dummy = 0            
    buffer = ser.read( Nget*(2+n_trace)*4)
    N = len(buffer) / ((2+n_trace)*4)
    n = np.zeros( int(N) , dtype=np.uint32 )
    t = np.zeros( int(N) ) 
    s = np.zeros( ( int(N) , Nsig-2 ))
    sigtypes = ['I' , 'I' ] + ser.tracesigtypes
    for sigtype in set(sigtypes):
        indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
        indices = np.array(indices)
        if sigtype == 'b':
            tmp = np.ndarray( ( int(N) , Nsig ) , 'bool', buffer[0::4] )
        else:
            tmp = np.ndarray( ( int(N) , Nsig ) , sigtype, buffer )
        if sigtype == 'I':
            n = tmp[:,indices[0]]
            t = tmp[:,indices[1]] / 1e6
            s[:,indices[2:]-2] = tmp[:,indices[2:]]
        else:
            s[:,indices-2] = tmp[:,indices]

 

    T.extend( t[:] )
    y1.extend( s[:,0]); 
    y1a.extend( s[:,1]); 
    y2.extend( s[:,2] ); 
    y3.extend( s[:,3] ) ; 
    y4.extend( s[:,4] ) ; 
    
    while len(y1) > 3000:
        y1.popleft() #remove oldest
        y1a.popleft() #remove oldest
        y2.popleft() #remove oldest
        y3.popleft()
        y4.popleft()
        T.popleft()
    curve1.setData( x=T , y=y1)
    curve1a.setData( x=T , y=y1a)
    curve2.setData( x=T , y=y2)
    curve3.setData( x=T , y=y3)
    curve4.setData( x=T , y=y4)
    app.processEvents()  

 

setpar('Nsend' , 0)
bla = ser.readall()

#%% test
setpar('Nsend' , 100)
buff = ser.readall()
 
 
 #%% test PC state feeback
setpar('EnStateFB',0)
setpar('EncPos1Raw',0)
setpar('EncPos2Raw',2500) # Downward position = -pi
setpar('PCstate',2)

#%% test PC state feeback on
setpar('EnStateFB',400)

#%% test PC off
setpar('PCstate',0)

#%% test PC cascade controller
setpar('EnStateFB',0)
setpar('EncPos1Raw',0)
setpar('EncPos2Raw',2500) # Downward position = -pi
setcont( 3 )
#%% test PC cascade controller
setpar('PCstate',3)
