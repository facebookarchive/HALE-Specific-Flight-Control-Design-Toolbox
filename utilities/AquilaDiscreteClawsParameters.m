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


      
% Tau refers to time constant of the filter
% Coe refers to coefficient which could be in numeriator (Num) or denominator (Den). Note that the highest order num/den coefficient is always assumed to be unity. 

Ts = (1/25) ;

AP_PitchLoop_Kp = 0.4;


AP_PitchLoop_Kd = 0.0;     

AP_PitchLoop_Ki = 0.1;      

AP_PitchLoop_Kff = 0.0;     

AP_PitchLoop_DerivLPFDenTau = 0.1;

AP_PitchLoop_Kq = 0.1;        


% AP_PitchLoop_qFdbkLLFiltNumCoe = -0.688 ;
% AP_PitchLoop_qFdbkLLFiltDenCoe = -0.986 ;
AP_PitchLoop_qFdbkLLFiltNumCoe = -0.87 ;
AP_PitchLoop_qFdbkLLFiltDenCoe = -0.94 ;

AP_PitchLoop_qFdbkLFLLFiltNumCoe = -0.9;
AP_PitchLoop_qFdbkLFLLFiltDenCoe = -0.9;


AP_PitchLoop_wn_ElvRollOff = 15 ;                   %Equivalent to 2.5Hz of the experimented servo bandwidth
AP_PitchLoop_zeta_ElvRollOff = 0.9 ;
 
AP_PitchLoop_thetaCmdLPFDenTau = 0.1 ;              %10 rad/sec bandwidth

%Pitch loop Notch filter 1 parameters
AP_PitchLoop_Notch1BWfactor         =   0.8631 ;                                  %decrease it to 0.7-0.75 range to increase the notch bandwidth - could be constant for all the notchs across the flight envelop
AP_PitchLoop_Notch1Freq             =   36.3/(2*pi) ;
AP_PitchLoop_Notch1FreqNorm         =   (AP_PitchLoop_Notch1Freq*Ts)*2*pi ;
AP_PitchLoop_Notch1CosFreqNorm      =   cos(AP_PitchLoop_Notch1FreqNorm);          %this parameter is scheduled across the flight envelop

%choose 1 to use the MATLAB IIR 2nd order notch filter, 
%choose 0 to use the in-house IIR 2nd order notch filter - used when
%scheduling the filter notch frequency arcoss the flight envelop
UseMatlabIIRNotchFilt = 1 ;


%%
%Climb Rate loop
AP_ClimbRateLoop_MaxPitchCmdrad = (20)*pi/180 ;
AP_ClimbRateLoop_MinPitchCmdrad = (-15)*pi/180 ;

AP_ClimbRateLoop_Kp = 0.05 ;
AP_ClimbRateLoop_Ki = 0.0 ;

AP_ClimbRateLoop_LLFiltNumCoe = -0.9;
AP_ClimbRateLoop_LLFiltDenCoe = -0.9;


AP_ClimbRate_LPFDenTau = 0.1 ; 

AP_ClimbRateLoop_ClimbRateCmdLPFDenTau = 0.07 ;   %20 rad/sec bandwidth
%%
%TAS loop
AP_TASLoop_Kp = 0.08;
AP_TASLoop_Ki = 0.02;

AP_TASLoop_Kvadv = 0 ;

AP_TASLoop_MaxClimbRateCmdmps = 3;   %m/s
AP_TASLoop_MinClimbRateCmdmps = -3;  %m/s

AP_TASLoop_TASCmdLPFDenTau   = 0.25 ;   %  4 rad/sec  

AP_TASLoop_TASadvLPFDenTau   = 0.1  ;

AP_TASLoop_LLFiltNumCoe = -0.9;
AP_TASLoop_LLFiltDenCoe = -0.9;

AP_TASLoop_ClimbRateCmdRateLimMax   = 0.35  ;     %m/s/s 0.05g = 0.5 m/s/s
AP_TASLoop_ClimbRateCmdRateLimMin   = -0.35  ;

%%
%Altitude Loop
AP_AltLoop_Kp = 0.6;
AP_AltLoop_Ki = 0.025 ;
AP_AltLoop_Kd = 0 ;

AP_AltLoop_Ki_WindupMaxLim = 0.5 ;   %
AP_AltLoop_Ki_WindupMinLim = -0.5 ;

AP_AltLoop_AltCmdLPFDenTau = 1 ;  % 1 rad/sec

AP_AltLoop_altRateCmdMaxLimMps = 0.3;     %(1fps) - For the slew limit on the altitude command (from the Guidance loop)
AP_AltLoop_altRateCmdMinLimMps = -0.3;
%%
%Power loop
AP_PwrLoop_Kp = 0.5 ;
AP_PwrLoop_Ki = 0.0 ;

AP_PwrLoop_Kvadv = 0.5;

AP_PwrLoop_LLFiltNumCoe = -0.9;
AP_PwrLoop_LLFiltDenCoe = -0.9;

AP_PwrLoop_TASadvLPFDenTau = 0.1 ;

AP_PwrLoop_ClimbRateCmdLPFDenTau = 0.1 ;

AP_PwrLoop_MaxThrCmdNorm = 1 ; 
AP_PwrLoop_MinThrCmdNorm = -1 ;      % Note that in the linear design, the AP_PwrLoop_MinThrCmdNorm could be negative meaning reduction in the throttle command w.r.t the trim throttle, but in the nonlinear sense, the throtle command cannot go under zero

AP_PwrLoop_wn_ThrRollOff   = 9 ;
AP_PwrLoop_zeta_ThrRollOff = 0.9 ;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Roll/rate Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------
%Roll to Yaw loop

AP_RollToYawRateLoop_Kp = 0.3 ;                     %Approx:  g/V_T, the coupling term between the bank attitude error and the yaw rate command
AP_RollToYawRateLoop_Ki = 0.05 ;                   

AP_RollToYawRateLoop_YawRateWindupMaxLim = 1.5 ;    %Dps
AP_RollToYawRateLoop_YawRateWindupMinLim = -1.5 ;

%----------------
%Roll rate damper
AP_RollRateLoop_Kp = 0;                             

AP_RollLoop_pFdbkLLFiltNumCoe = -0.68 ;
AP_RollLoop_pFdbkLLFiltDenCoe = -0.68 ;

AP_RollLoop_pFdbkLFLLFiltNumCoe = -0.9 ;
AP_RollLoop_pFdbkLFLLFiltDenCoe = -0.9 ;

%----------------
%Roll loop
AP_RollLoop_Kp = 1;
AP_RollLoop_Ki = 0.0;       

AP_RollLoop_rollAdvLLFiltNumCoe = -0.9 ;
AP_RollLoop_rollAdvLLFiltDenCoe = -0.9 ;

AP_RollLoop_phiErrLPFDenTau = 0.1 ;

AP_RollLoop_ailCmdMaxLim = 15 ;                    % Max aileron deflection limit
AP_RollLoop_ailCmdMinLim = -15 ;                   % Min aileron deflection limit 

AP_RollLoop_DerivLPFDenTau = 0.1;

AP_RollLoop_KrollrateAdv   = 0 ;                    %Damping term on the roll attitude error loop -  


AP_RollLoop_wn_AilRollOff   = 12 ;                %Equivalent to 1.5Hz of the experimented servo bandwidth
AP_RollLoop_zeta_AilRollOff = 0.9 ;
 
AP_RollRateLoop_rateCmdLPFDenTau = 0.1 ;               % 10 rad/sec bandwidth

AP_RollLoop_AilWindupMaxLim = 10 ;                %max Aileron windup limit in Deg - should be a percentage of the total Aileron deflection
AP_RollLoop_AilWindupMinLim = -10 ;               %min Aileron windup limit in Deg


%Roll loop Notch filter 1 parameters
AP_RollLoop_Notch1BWfactor         =   0.8631 ;                                  %decrease it to 0.7-0.75 range to increase the notch bandwidth - could be constant for all the notchs across the flight envelop
AP_RollLoop_Notch1Freq             =   36.3/(2*pi) ;
AP_RollLoop_Notch1FreqNorm         =   (AP_RollLoop_Notch1Freq*Ts)*2*pi ;
AP_RollLoop_Notch1CosFreqNorm      =   cos(AP_RollLoop_Notch1FreqNorm);          %this parameter is scheduled across the flight envelop

%%
%GndTrk Loop
AP_GndTrkLoop_Kp = 0.05;
AP_GndTrkLoop_Kd = 0.0;      

AP_GndTrkLoop_phiCmdMaxLimDeg = 5 ;            %max phi command command 5 deg
AP_GndTrkLoop_phiCmdMinLimDeg = -5 ;      

AP_GndTrkLoop_phiRateCmdMaxLimDps = 1.5 ;       %max phi rate command command 1.5 dps
AP_GndTrkLoop_phiRateCmdMinLimDps = -1.5 ;      


AP_GndTrkLoop_CmdLPFDenTau = 0.1 ;


%%
%Yaw/rate Loop
AP_YawRateLoop_Kp = 0.2 ;
AP_YawRateLoop_Ki = 0.0 ;                         %0.02
AP_YawRateLoop_Kd = 0.0 ;                         %do not use the Kd term in the Yaw rate loop, it would be noisy, instead use the AP_YawRateLoop_KpAdv 

AP_YawRateLoop_LLFiltNumCoe = -0.95 ;
AP_YawRateLoop_LLFiltDenCoe = -0.95 ;

AP_YawRateLoop_LFLLFiltNumCoe = -1 ;
AP_YawRateLoop_LFLLFiltDenCoe = -1 ;

AP_YawRateLoop_RdrCmdMaxLimNorm = 0.25 ;        %max Rdr command command norm 0.25 when diffrential throttle is used for directional control
AP_YawRateLoop_RdrCmdMinLimNorm = -0.25 ;      

AP_YawRateLoop_YawRateMaxLimDps = 3.5 ;         %max yaw rate command (deg/s), function of max bank attitude and g/V_T
AP_YawRateLoop_YawRateMinLimDps = -3.5 ;        %min yaw rate command (deg/s)

AP_YawRateLoop_RdrWindupMaxLim = 0.15 ;         %max Rdr windup limit Norm - should be a percentage of the total differential throttle deflection
AP_YawRateLoop_RdrWindupMinLim = -0.15 ;        %min Rdr windup limit Norm


AP_YawRateLoop_ErrLPFDenTau = 0.1 ;            %time constant on the yaw rate error LPF

AP_YawRateLoop_wn_DiffThrRollOff   = 12 ;       %rad/sec, almost 2 Hz
AP_YawRateLoop_zeta_DiffThrRollOff = 0.9 ;

%% Flex Mode(s) control parameters
FlexControlActive = 0;

AP_FlexLoop_nzFdbk_Kp = 0.015 ;

AP_FlexLoop_nzFdbkLLFiltNumCoe = -0.8975 ;
AP_FlexLoop_nzFdbkLLFiltDenCoe = -0.6544 ;

AP_FlexLoop_NotchDampingNum         =   0.2 ;                                  %Damping of the numerator
AP_FlexLoop_NotchDampingDen         =   0.6 ;                                  %Damping of the denominator
AP_FlexLoop_NotchFreq               =   15.7 ;                                 %Notch Frequency in rad

AP_FlexLoop_wn_ElvRollOff           =   50 ;                                   % A lag roll-off filter in the Flex loop to diminish the high frequency components of the Flex loop

%% GLA control parameters
GLAControlActive = 0 ;

AP_GLALoop_nzFdbk_Kp = 0.002;
AP_GLALoop_nzFdbk_Ki = 0;

AP_GLALoop_nzFdbkDenTau =  1/12;            %time constant on the Nz feedback LPF in sec - 12rad/sec


AP_GLALoop_MaxSplCmdDeg = 8 ;    %in deg
AP_GLALoop_MinSplCmdDeg = 0 ;    %in deg

