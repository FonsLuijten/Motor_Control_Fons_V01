#%%Definitions
"""
Created on Sat Sep 29 19:25:07 2018

@author: Elwin + Fons
"""

#### Imports

from os import chdir
#chdir('C:\\Users\\fonsl\\OneDrive\\Projecten\\Mavilor motors\\Software\\Motor_Control_Fons_V01\\Python')
chdir('D:\\OneDrive\\Documenten\\Projecten\\Mavilor motors\\Software\\Motor_Control_Fons_V01\\Python')

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

from numpy import (atleast_1d, poly, polyval, roots, real, asarray,
                   resize, pi, absolute, logspace, r_, sqrt, tan, log10,
                   arctan, arcsinh, sin, cos, exp, cosh, arccosh, ceil, conjugate,
                   zeros, sinh, append, concatenate, prod, ones, array,
                   mintypecode)
from numpy.polynomial.polynomial import polyval as npp_polyval
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)


#### Settings

#f_pwm = 20e3
Ts = 50e-6 #1/(2*f_pwm)
n_trace = 10;
com = 'COM3'

#### Start serial communication

try:
    ser = serial.Serial( com  , timeout= 0.1 )  # open serial port 0.1
    print(ser.name)         # check which port was really used
except:
    print(ser.name)         # check which port was really used


#### Definitions

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
    return ser

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
        #print('set par')
        for ii in range(length):
            signal = ser.signames.index( signals[ii] )
            value = values[ii]
            #print(signal)
            #print(value)
            #print(ser.sigtypes[signal])
            ser.write( b'S' + struct.pack('I',  signal) + struct.pack( ser.sigtypes[signal] ,  value)  )

def getpar( signals ):
    s = getsig( signals )
    return(s)

def getsig( signals ):
    if isinstance( signals , str):
        signals = [ signals ];
    signals = setTrace( signals )
    t,s = readData( 1 )
    
    #print(s)
    
    if len(signals) == 1:
        s = s[0,0];
    else:
        s = s[0,0:len(signals)];
#    print( s)
    return(s)

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
    #print('set trace')
    for signal in ser.signalsnames:
        if isinstance( signal , str):
            signal = ser.signames.index( signal )
        ser.signals[i] = signal
        ser.tracesigtypes[i] = ser.sigtypes[signal]
        #print(signal)
        ser.write( b't' + struct.pack('I', signal) + bytes(bytearray([i])) )
        i+=1
    return signals

def readData(N):
    Nsig = 2+n_trace
    setpar('Nsend' , N)
    if N < 100:
        buffer = ser.read(N*4*Nsig)
    else:
        buffer = ser.readall()
        
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


def prepSP( spn, p , v , a , j ):
    t1, t2, t3, jd = make3.make3( p , v , a , j , Ts );
    #ser.write( b'1' + struct.pack('f',  t1) )
    #ser.write( b'2' + struct.pack('f',  t2) )
    #ser.write( b'3' + struct.pack('f',  t3) )
    #ser.write( b'4' + struct.pack('f',  p) )
    #ser.write( b'5' + struct.pack('f',  v) )
    #ser.write( b'6' + struct.pack('f',  a) )
    #ser.write( b'7' + struct.pack('f',  jd) )
    setpar('sp'+str(spn)+'_t1',t1)
    setpar('sp'+str(spn)+'_t2',t2)
    setpar('sp'+str(spn)+'_t3',t3)
    setpar('sp'+str(spn)+'_p',p)
    setpar('sp'+str(spn)+'_vmax',v)
    setpar('sp'+str(spn)+'_amax',a)
    setpar('sp'+str(spn)+'_jmax',j)
    setpar('spSET',1)
    return t1, t2, t3, jd


def getFFT( signal , Ndownsample ):
    j0 = int(1/Ts) # These amount of samples are ignored
    L = 2047 * Ndownsample
    Naver = int( (len(signal)-j0)/L  )
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
    plt.subplot(2,1,2, sharex=ax1) #ax2 = 
    plt.semilogx( f , np.cumsum(abs(SIGNAL)**2 /Ts) *Ts )
    plt.grid( 1 , 'both')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('CPS [-²]')
    plt.tight_layout()
    plt.show()   

    return SIGNAL, f

def freqrespd( num , den ,f , Ts ):
    zm1 = exp(-1j * 2*np.pi * f*Ts )
    H = (npp_polyval(zm1, num, tensor=False) / npp_polyval(zm1, den, tensor=False))
    return H

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
    plt.subplot(2,1,2, sharex=ax1) #ax2 = 
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
# 0: Amplifiers + PWM off
# 1: Set direct PWM
# 2: Homing encoders (by setting direct PWM)
# 3: Commutation angle riding the waves
# 4: Current control only open loop (direct setting of Q voltage, can be used for identification of current loop)
# 5: Current control only closed loop
# 6: Position control closed loop


#%% OK Start Serial communication
ser = start(ser)

#%% OK Stop Serial communication
ser.close()

#%% OK Close all figures
plt.close('all')

#%% OK Restart Controller
setpar('errcode',999)

#%% OK Get signal
signals = ['errcode']
s = getsig( signals )
print(s)

#%% OK Read and plot data
signals = [ 'Xm1']#,'ThetaE1']
signals = setTrace( signals )
t,s = readData( int(10/Ts) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()

#%% OK state 0
setpar( 'state' , 0)

#%% OK direct setting PWM
PWM = [0.5,0.5,0.5]
PWM = [0,0,0]
setpar( 'state' , 1)
setpar( 'PWMph1offs' , PWM)

#%% --- Homing and calibration ---

#%% OK Zeroing encoders through OL setting PWM 6 step
setpar( 'state' , 2) 

Vbus = getpar('V_bus');
Vhome = 2;
PWMoff = [[-1,1,1], [-1,-1,1], [1,-1,1], [1,-1,-1], [1,1,-1], [-1,1,-1]]
PWMoff = np.dot(PWMoff, 0.5*Vhome/Vbus)
PWMoff = np.add(PWMoff, 0.5)
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

#%% OK Calibration of commutation angle, riding the waves
freq = 0.5; # frequency for riding the waves
ampl = 3;   # amplitude Q voltage for riding waves

setpar('ThetaEcalAmp',ampl)
setpar('ThetaEcalFreq',freq)

nsamp = int(10*1/freq/Ts); # 10 periods @freq Hz

setpar( 'state' , 3)
time.sleep(1)
signals = [ 'ThetaE1','ThetaE1cal','Vph1dq']
signals = setTrace( signals )
t,s = readData( nsamp )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
    
setpar( 'state' , 0)    

ThetaRef,f = getFFT_SS( s[:,0]  )
ThetaCal,f = getFFT_SS( s[:,1]  )
ThetaResp = ThetaRef/ThetaCal;

angle = np.angle(ThetaResp[f == freq])+np.pi;
print(angle)

setpar('ThetaE1cal', angle)

s[:,1] = s[:,1]-np.mean(s[:,1]);
plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()

plt.figure();
plt.plot(s[:,0],s[:,1], );
plt.legend( signals )
plt.show()

plt.figure()
bode( ThetaResp , f , 'ThetaRef/ThetaCal')

#%% --- Current loop ---

#%% OK Open current loop constant voltage
setpar( 'state' , 4)
setpar( 'Vph1dqOffs' ,[0,3] )
setpar('enSVM',1)

signals = [ 'Im1','Vph1']
signals = [ 'Xm1','Vph1dq[1]']
signals = setTrace( signals )
t,s = readData( int(1/Ts) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')

setpar( 'state' , 0)
setpar( 'Vph1dqOffs' , [0,0])

plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()

#%% OK SingleSine Open current loop identification
ss_fstart = 10;
ss_fstep = 20;
ss_fend = 2000;
ss_n_aver = 100;
ss_distgain = 0.1;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)

print( t_tot )

N = int( t_tot / Ts)

setpar( 'state' , 0)
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

setpar( 'state' , 4)
setpar('endistcurD',0)
setpar('endistcurQ',1)

time.sleep(0.5)

setpar('ss_gain',ss_distgain)
signals = ['ss_f','dist','Im1dq','Vph1']
signals = setTrace( signals )
t,s = readData( int(N) )
setpar('ss_gain',0)


setpar('endistcurD',0)
setpar('endistcurQ',0)
setpar( 'state' , 0)
setpar( 'distoff' , 0)        # disturbance offset

for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
Pd = np.zeros( len(freqs) ,dtype=np.complex_)
Pq = np.zeros( len(freqs) ,dtype=np.complex_)

for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')])
    IM1DQ_0 , f = getFFT_SS(  s[I, signals.index('Im1dq[0]')] )
    IM1DQ_1 , f = getFFT_SS(  s[I, signals.index('Im1dq[1]')] )
    index = np.argmin( abs(freq-f ))
    Pd[freq==freqs] = IM1DQ_0[index] / DIST[index]  
    Pq[freq==freqs] = IM1DQ_1[index] / DIST[index]  

plt.figure(1)
bode( Pd , freqs , 'Pd')
plt.figure(2)
bode( Pq , freqs , 'Pq')

plt.figure();
plt.plot(t,s[:,1:len(signals)], );
plt.legend( signals[1:len(signals)] )
plt.show()

#%% OK PRBS open  Current loop FRF measurement 
Ndownsample = 1
N = 10*2047 + 1/Ts
setpar( 'state' , 4)
setpar( 'Ndownsample' , Ndownsample) #Downsampling
setpar( 'prbs_gain' , 0.5) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar('endistcurD',1)
setpar('endistcurQ',1)

signals = ['dist','Im1dq']
signals = setTrace( signals )
t,s = readData( int(N) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')


setpar('endistcurD',0)
setpar('endistcurQ',0)
setpar( 'state' , 0)
setpar( 'prbs_gain' , 0) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar( 'Ndownsample' , 1) #Downsampling

DIST,f = getFFT( s[:, signals.index('dist')] , Ndownsample )
IM1DQ_0 , f = getFFT( s[:, signals.index('Im1dq[0]')] , Ndownsample )
IM1DQ_1 , f = getFFT( s[:, signals.index('Im1dq[1]')] , Ndownsample )

Pd = IM1DQ_0 / DIST
Pq = IM1DQ_1 / DIST

plt.figure(1)
bode( Pd , f , 'Pd')
plt.figure(2)
bode( Pq , f , 'Pq')

#%% OK open current loop music
setpar( 'state' , 4)
setpar('endistcurQ',1)
setpar( 'distoff' , -0.5)
setpar( 'music_gain' , 5) # music current

#%% OK turn off music
setpar( 'state' , 0)
setpar('endistcurQ',0)
setpar( 'distoff' , 0)
setpar( 'music_gain' , 0) # music current

#%% OK Turn on current control
setpar( 'state' , 5)

#%% OK Closed current loop identification
ss_fstart = 10;
ss_fstep = 20;
ss_fend = 2000;
ss_n_aver = 100;
ss_distgain = 1;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)

print( t_tot )

N = int( t_tot / Ts)

setpar( 'state' , 0)
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

setpar( 'state' , 5)
setpar('endistcurQ',1)

time.sleep(0.5)

setpar('ss_gain',ss_distgain)
signals = ['ss_f','dist','Vph1dq','Im1dq']
signals = setTrace( signals )
t,s = readData( int(N) )
setpar('ss_gain',0)


setpar('endistcurQ',0)
setpar( 'state' , 0)
setpar( 'distoff' , 0)        # disturbance offset

for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
S = np.zeros( len(freqs) ,dtype=np.complex_)
PS = np.zeros( len(freqs) ,dtype=np.complex_)
P = np.zeros( len(freqs) ,dtype=np.complex_)
C = np.zeros( len(freqs) ,dtype=np.complex_)
OL = np.zeros( len(freqs) ,dtype=np.complex_)

for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')])
    VPH1Q_1 , f = getFFT_SS(  s[I, signals.index('Vph1dq[1]')] )
    IM1DQ_1 , f = getFFT_SS(  s[I, signals.index('Im1dq[1]')] )
    index = np.argmin( abs(freq-f ))
    S[freq==freqs] = VPH1Q_1[index] / DIST[index]  
    PS[freq==freqs] = IM1DQ_1[index] / DIST[index]  

P = PS/S
C = (1/S-1)/P  
OL = P*C 

plt.figure(1)
bode( P , freqs , 'P')
plt.figure(2)
bode( C , freqs , 'C')
plt.figure(3)
bode( OL , freqs , 'OL')

plt.figure();
plt.plot(t,s[:,1:len(signals)], );
plt.legend( signals[1:len(signals)] )
plt.show()

#%% --- Position loop ---

#%% OK PRBS Open position loop identification
Ndownsample = 1
N = 10*2047 + 1/Ts
setpar( 'state' , 5)
setpar( 'Ndownsample' , Ndownsample) #Downsampling
setpar( 'prbs_gain' , 0.3) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar('endistcurSP',1)

signals = ['dist','Xm1']
signals = setTrace( signals )
t,s = readData( int(N) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')

setpar('endistcurSP',0)
setpar( 'state' , 0)
setpar( 'prbs_gain' , 0) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar( 'Ndownsample' , 1) #Downsampling

Kt = getpar('Kt1')

DIST,f = getFFT( Kt*s[:, signals.index('dist')] , Ndownsample )
XM1 , f = getFFT( s[:, signals.index('Xm1')] , Ndownsample )

P = XM1 / DIST

plt.figure(1)
bode( P , f , 'P')

#%% OK SingleSine Open position loop identification
ss_fstart = 10;
ss_fstep = 20;
ss_fend = 2000;
ss_n_aver = 100;
ss_distgain = 0.05;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)

print( t_tot )

N = int( t_tot / Ts)

setpar( 'state' , 0)
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

setpar( 'state' , 5)
setpar('endistcurSP',1)

time.sleep(0.5)

setpar('ss_gain',ss_distgain)
signals = ['ss_f','dist','Xm1']
signals = setTrace( signals )
t,s = readData( int(N) )
setpar('ss_gain',0)


setpar('endistcurSP',0)
setpar( 'state' , 0)
setpar( 'distoff' , 0)        # disturbance offset

for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
P = np.zeros( len(freqs) ,dtype=np.complex_)

Kt = getpar('Kt1')

for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( Kt*s[I, signals.index('dist')])
    XM1 , f = getFFT_SS(  s[I, signals.index('Xm1')] )
    index = np.argmin( abs(freq-f ))
    P[freq==freqs] = XM1[index] / DIST[index]  

plt.figure(1)
bode( P , freqs , 'P')

plt.figure();
plt.plot(t,s[:,1:len(signals)], );
plt.legend( signals[1:len(signals)] )
plt.show()

#%% OK Turn on Position control close loop
setpar('Kp_PC1',17.7)
setpar('Jload',5.61e-04)

setpar('spGO',0)
setpar('state',6)

#time.sleep(10)
#setpar('state',0)

#getpar('errcode')


#%% OK Load and play setpoint
p = 6.28
v = 200
a = 200
j = 100

[t1, t2, t3, jd] = prepSP( 1, p , v , a , j )

signals = [ 'Xsp1', 'Xm1' , 'Xe1']
setTrace( signals )

setpar('spGO',1)
N = int( (1.2*(0.01+4*t1+2*t2+t3)+0.1) / Ts ) 
t,s = readData( int(N) )
setpar('spGO',0)

plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.grid( 1 , 'both')
plt.show()


#%% OK Move to zero
p = -1*getsig('Xm1')
v = 200
a = 200
j = 100

[t1, t2, t3, jd] = prepSP( 1, p , v , a , j )

signals = [ 'Xsp1', 'Xm1' , 'Xe1']
setTrace( signals )

setpar('spGO',1)
N = int( (1.2*(0.01+4*t1+2*t2+t3)+0.1) / Ts ) 
t,s = readData( int(N) )
setpar('spGO',0)

plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.grid( 1 , 'both')
plt.show()

#%% OK Open Brake
setpar('enBrake',1) # Open
#setpar('enBrake',0) # Close

#%% OK Reset Error
setpar('errcode',999)7i

#%% OK Music position control
setpar( 'distoff' , 0)
setpar( 'music_gain' , 0.2) # music current
setpar('endistpos',1)

#%% OK Turn off Music position control
setpar('endistpos',0)
setpar( 'distoff' , 0)
setpar( 'music_gain' , 0) # music current


#%% OK Single sine closed Position loop identification
ss_fstart = 5;
ss_fstep = 1;
ss_fend = 200;
ss_n_aver = 20;
ss_distgain = 0.2;
ss_distoffset = 0;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )

N = int( t_tot / Ts)

#setpar( 'Kp1' , 0) # Kp1 = 0
setpar( 'offsetVel' , 0.0*np.pi)

setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

#setpar( 'IntegOn' , 0)

setpar('endistpos',1)

time.sleep(0.5)

setpar('ss_gain',ss_distgain)
signals = [ 'ss_f' , 'dist' ,'PC1out' ,'Xe1']
setTrace( signals )
t,s = readData( int(N) )
for signal in signals:
    varname = signal.replace("[", "_");
    varname = varname.replace("]", "");
    exec(varname + ' = s[:, signals.index(\''+signal+'\')]')
setpar('ss_gain',0)

setpar( 'distoff' , 0)
setpar( 'offsetVel' , 0.0*np.pi)
setpar('endistpos',0)

#setpar( 'IntegOn' , 1)

ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
S = np.zeros( len(freqs) ,dtype=np.complex_)
PS = np.zeros( len(freqs) ,dtype=np.complex_)

for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')] )
    PC1OUT , f = getFFT_SS( s[I, signals.index('PC1out')] )
    XE1 , f = getFFT_SS( s[I, signals.index('Xe1')] )

    index = np.argmin( abs(freq-f ))  

    S[freq==freqs] = PC1OUT[index] / DIST[index]
    PS[freq==freqs] = -XE1[index] / DIST[index]    

plt.close('all')
plt.figure(1)
bode( PS/S , freqs , 'P')


plt.figure(3)
bode( 1/S-1 , freqs , 'OL')
plt.figure(4)
nyquist( 1/S-1 , freqs , 'OL')

plt.figure(5)
bode( (1/S-1)/(PS/S) , freqs , 'C')

plt.figure(6)
bode( S , freqs , 'S')

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
