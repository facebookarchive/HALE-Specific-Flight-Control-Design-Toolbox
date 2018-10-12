%   Copyright (C) 2018-present, Facebook, Inc.
% 
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; version 2 of the License.
% 
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
% 
%   You should have received a copy of the GNU General Public License along
%   with this program; if not, write to the Free Software Foundation, Inc.,
%   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
%
%% Design the Notch
%define the first Notch filter frequency
fs = 25; %Hz

fo = 5.23/(2*pi);   % fo in Hz --> 36rad/sec or 36.3rad/sec for the longitudinal loop, 39rad/sec for the lateral loop , for higher altitudes such as 18km, 
                   % another notch is required at the first bending mode frequency at 4.8rad/sec ~ 0.76 Hz
Q_factor = 5;

wo = fo/(fs/2);

bw = wo/Q_factor;


[num_notch,den_notch] = iirnotch(wo,bw);
Gd_notch = tf(num_notch,den_notch,(1/fs))
fvtool(num_notch,den_notch);

figure(110);hold on; bode(Gd_notch,'k');
figure(109);hold on; pzmap(Gd_notch,'b');

%define the second Notch filter frequency
%%
fo = 39.2/(2*pi);   %fo in Hz 36rad/sec
Q_factor = 5;

wo = fo/(fs/2);

bw = wo/Q_factor;


[num_notch_2,den_notch_2] = iirnotch(wo,bw);
Gd_notch_2 = tf(num_notch_2,den_notch_2,(1/fs))
fvtool(num_notch_2,den_notch_2);



%%Analysis
% AP_PitchLoop_qFdbkFiltDenCoe = -0.987
% -----------------------------------
% AP_PitchLoop_qFdbkFiltNumCoe = -0.536
% %Design a continuous filter

w_n = 36.3 ;
zeta1 = 0.01
zeta2 = 0.9 

num_notch_new = [ 1 2*zeta1*w_n w_n^2 ];
den_notch_new = [ 1 2*zeta2*w_n w_n^2 ];

% G_notch_new = tf(num_notch_new,den_notch_new,0.04);
Gc_notch = tf(num_notch_new,den_notch_new);

figure(110);hold on;bode(Gc_notch,'k');
Gd_notch_c2d = c2d(Gc_notch,(1/fs),'Tustin',c2dOptions('PrewarpFrequency',w_n))

figure(110);hold on;bode(Gd_notch_c2d,'m');
figure(109);hold on; pzmap(Gd_notch_c2d);
legend('Discrete filter design','Discretized with Tustin',...
    'Location','NorthWest')


%% analyze the designed notch filter charachteristics
AP_PitchLoop_ElvNotchFiltNum = [0.872 -0.2068 0.8724];   %poles are at 0.1 + 0.85i
AP_PitchLoop_ElvNotchFiltDen = [1     -0.2068 0.7449];

G_notch= tf(AP_PitchLoop_ElvNotchFiltNum,AP_PitchLoop_ElvNotchFiltDen,0.04);
damp(G_notch), zpk(G_notch), zero(G_notch)
figure(110);pzmap(G_notch);
figure(100);hold on;bode(G_notch,'k');

% Pure FIR Notch filter, no anti-aliasing
% num = 1 - (2*cosw0)*z^-1 + z^-2
% den = 1 - (2*r*cosw0)*z^-1 + (r^2)*z^-2
w0 = 36.3/(2*pi)
r = 0.85;
rf = 0.9945;
ntchD2 = tf([1  -(2*cos(rf*w0)) 1],[1 -2*r*cos(rf*w0) r^2],1/25);
[mD2,pD2] = bode(ntchD2,w);
mD2 = reshape(mD2,length(mD2),1);
pD2 = reshape(pD2,length(pD2),1);

% ntchD2 =
%  
%     z^2 + 0.02568 z + 1
%   ------------------------
%   z^2 + 0.02183 z + 0.7225
% 
% Sample time: 0.04 seconds
% Discrete-time transfer function.

subplot(211);
semilogx(w,20*log10(mD2),'g');
subplot(212);
semilogx(w,pD2,'g');

% FIR Notch filter, with anti-aliasing
[D] = fir_dsn(39.5/(fs*2),r,15);
[h,f] = freqz(D,1,npts,'whole',fs);
subplot(211);semilogx(f*fs/2,20*log10(abs(h)),'m');
subplot(212);semilogx(f*fs/2,phase(h)*180/pi,'m');

subplot(211);
ylabel('Magnitude [dB]');
legend('Continuous','Discrete IIR','Notch FIR 1','Notch FIR 2');
subplot(212);
xlabel('Frequency [rad/sec]');


%%Notch filter design
Ts = (1/25) ;
fs=25 ;
NotchBWfactor = 0.75;
NotchFreq  = 36.3/(2*pi) ;
NotchFreqNorm = (NotchFreq/fs)*2*pi ;

num_Notch = (0.5*(1+NotchBWfactor))*[ 1 -2*cos(NotchFreqNorm) 1 ]
den_Notch = [ 1 -2*NotchBWfactor*cos(NotchFreqNorm) (NotchBWfactor)^2 ]

G_Notch = tf(num_Notch,den_Notch,1/fs)
figure(112);bode(G_Notch,'r',G_notch_,'b');

%% analyze the designed notch filter charachteristics
AP_PitchLoop_ElvNotchFiltNum = [0.872 -0.2068 0.8724];   %poles are at 0.1 + 0.85i
AP_PitchLoop_ElvNotchFiltDen = [1     -0.2068 0.7449];

G_notch_= tf(AP_PitchLoop_ElvNotchFiltNum,AP_PitchLoop_ElvNotchFiltDen,0.04);


AP_PitchLoop_Notch1BWfactor         =   0.8631 ;
AP_PitchLoop_Notch1Freq             =   36.3/(2*pi) ;
AP_PitchLoop_Notch1FreqNorm         =   (AP_PitchLoop_Notch1Freq*Ts)*2*pi ;
AP_PitchLoop_Notch1CosFreqNorm      =   cos(AP_PitchLoop_Notch1FreqNorm);

num_Notch = (0.4697*(1+AP_PitchLoop_Notch1BWfactor))*[ 1 -2*AP_PitchLoop_Notch1CosFreqNorm 1 ]
den_Notch = [ 1 -2*AP_PitchLoop_Notch1BWfactor*AP_PitchLoop_Notch1CosFreqNorm (AP_PitchLoop_Notch1BWfactor)^2 ]



% num_Notch = (0.5*(1+NotchBWfactor))*[ 1 -2*cos(NotchFreqNorm) 1 ]
% den_Notch = [ 1 -2*NotchBWfactor*cos(NotchFreqNorm) (NotchBWfactor)^2 ]

G_Notch = tf(num_Notch,den_Notch,1/fs)
figure(115);bode(G_Notch,'r',G_notch_,'b');



%% Roll loop Notch at 39.1rad/sec
AP_RollLoop_AilNotchFiltNum = [0.8638 -0.01174  0.8638];
AP_RollLoop_AilNotchFiltDen = [1      -0.01174  0.7276];


Gd_notch= tf(AP_RollLoop_AilNotchFiltNum,AP_RollLoop_AilNotchFiltDen,0.04);
figure(116);hold on;bode(Gd_notch,'r');

fs = 25;

fo = 39.1/(2*pi);   %fo in Hz 39.1rad/sec
Q_factor = 5;

wo = fo/(fs/2);

bw = wo/Q_factor;


[num_notch_2,den_notch_2] = iirnotch(wo,bw);
Gd_notch_2 = tf(num_notch_2,den_notch_2,(1/fs))
fvtool(num_notch_2,den_notch_2);


