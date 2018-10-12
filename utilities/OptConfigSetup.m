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
%
%	SYNTAX
%           [Opt] = OptConfigSetup(OptConfig)
%
%	PURPOSE: Setup and Configure the optimisation task including name of the tunable parameters, their initial condition, 
%            which loops to optimise,..., aircraft flight data name and the associated folder path
%
%	INPUTS:	For now just the OptConfig, which determines whether or not all the loops should be optimised concurrently. 
%           Put the Filter NumCoe and DenCoe beside each other, first
%           NumCoe, and then the DenCoe,
%           LLFilt stands for Lead/Lag filters, NotchFilt stands for Notch filters
%        
%
%	DEFAULT:
%
%	DEVELOPER :			H. Bolandhemmat (Facebook Connectivity Lab)
                      
%
%
function [Opt] = OptConfigSetup(OptConfig, OptimizeMultipleFlts)


Opt.ACModelName                 = 'Aquila_OL_Model_04_Jun_2018_10_35_21';           % Aquila model name built by the getAquilaModel_ModelGeneration.m script
Opt.UseCustomModel              = 1;                                                % Use aswing model
Opt.CustomModelName             = 'ss_0_8rom';                                        % Aswing ROM file name
Opt.ShowOpenLoopFltDyn          = 0 ;                                               % set to 1 (true) to review the aircraft open loop properties, location of sensors and servos and the mode shapes for the single flight condition (@Trim) - The information provided here is limited - run the AquilaFltDynAnalysis for complete information
Opt.PlotOpenLoopFltDyn          = 0 ;                                               % set to 1 to plot the open Loop Flight Dynamic bodes, step responses and pole/zero
Opt.PitchLoopOptSetting         = [10,1e-2];                                       % {'RandomStart','MinDecay'} ; define it for a contunious domain design, if discrete, Ts will be considered later
Opt.DesiredPitchLoopBW          = 0.5;   %in rad
Opt.OptModePlot                 = 'Off' ;
Opt.Write2XcelSheet             =  1;  
Opt.OutputXcelFileName          = 'ClawsOptOut.csv';                                %should be a mat file which contains the opt results, the control system performance measures as well as the flight conditions for which the control system has been designed.
Opt.InputXcelFileName           = 'FltCndInput.csv';

Opt.PlotSysIDPoles              = 0 ;                                               % Choose whether to plot comparison of identified poles to data
Opt.PlotSysIDFreq               = 0 ;                                               % Choose whether to plot identified input output response vs data

Opt.PitchLoopTuningGoals        = {'Freq_Shaping','Step_Tracking','Margins'};       %{'Freq_Shaping','Step_Tracking','Margins','Sensitivity','DisturbanceReject','Freq_Tracking','ClosedLoopPoles'};
Opt.DesiredClimbRateLoopBW      = 0.15;   %in rad
Opt.DesiredClimbRateLoopPM      = 45;     %in deg
Opt.DesiredClimbRateLoopGM      = 6;      %in dB


Opt.DesiredTASLoopBW            = 0.1;   %in rad

Opt.ClimbRateLoopTuningGoals    = {'Margins'};                                                 %{'Freq_Shaping','Step_Tracking','Margins'}; %The tuning goals for the Climb Rate Loop
Opt.TASLoopTuningGoals          = {'Freq_Shaping','Step_Tracking','Margins'};                  %{'Freq_Shaping','Step_Tracking'};%The tuning goals for the TAS Loop
Opt.DesiredTASLoopPM            = 45;     %in deg
Opt.DesiredTASLoopGM            = 6;      %in dB


Opt.DesiredPwrLoopBW            = 0.2;   %in rad
Opt.DesiredAltLoopBW            = 0.1;    %in rad

Opt.PwrLoopTuningGoals          = {'Margins'};                                              %{'Freq_Shaping','Step_Tracking','Margins'}; %The tuning goals for the Climb Rate Loop
Opt.DesiredPwrLoopPM            = 45;     %in deg
Opt.DesiredPwrLoopGM            = 6;      %in dB


Opt.AltLoopTuningGoals          = {'Freq_Shaping','Step_Tracking','Margins'};                   %{'Freq_Shaping','Step_Tracking'};%The tuning goals for the TAS Loop
Opt.RollLoopTuningGoals         = {'Freq_Shaping','Step_Tracking','Margins'};                                  %'Overshoot', The tuning goals for the Roll/rate Loop
Opt.GndTrkLoopTuningGoals       = {'Freq_Shaping','Step_Tracking','Margins'};                   %The tuning goals for the Ground Track control Loop

Opt.DesiredRollLoopBW           = 0.15;     %in rad
Opt.DesiredRollLoopOV           = 0.02 ;    %in percent
Opt.DesiredRollLoopPM           = 45 ;
Opt.DesiredRollLoopGM           = 6;


Opt.DesiredGndTrkLoopBW         = 0.05;    %in rad
Opt.DesiredGndTrkLoopPM         = 45;
Opt.DesiredGndTrkLoopGM         = 6;

Opt.YawRateLoopTuningGoals      = {'Margins'};                                                  %The tuning goals for the Yaw rate Loop

Opt.DesiredYawRateLoopPM        = 45;     %in deg
Opt.DesiredYawRateLoopGM        = 6;

Opt.Optimize4MultipleFlts       = OptimizeMultipleFlts ;

Opt.UsePrevOptResultAsIC        =  0;    % If this is set to zero, the code uses the standard pre-defined initial conditions for the control parameters/coeffs. However if it set to 1, then it uses the last round of tuned parameters stored in 'OptOut.mat'
Opt.getAllCrossovers            =  1;    % Some loops often have multiple places where they cross the 0db line. If this is set to 1 we get all of them in our output file and table and if its set to 0, we just get the last one.

%PitchLoop tunable parameters
optvars_PitchLoop          =   {'AP_PitchLoop_Kp','AP_PitchLoop_Ki','AP_PitchLoop_Kq'};      %'AP_PitchLoop_qFdbkLFLLFiltNumCoe','AP_PitchLoop_qFdbkLFLLFiltDenCoe'

optvarsMin_PitchLoop       =   [0.05,    0.05,    0.05] ;
optvarsMax_PitchLoop       =   [1,       1.5,     1] ;

%ClimbRateLoop tunable parameters
optvars_ClimbRateLoop          =   {'AP_ClimbRateLoop_Kp'};
optvarsMin_ClimbRateLoop       =   [0.001] ;
optvarsMax_ClimbRateLoop       =   [0.5] ;


%TAS loop tunable parameters
optvars_TASLoop          =   {'AP_TASLoop_Kp','AP_TASLoop_Ki'};
optvarsMin_TASLoop       =   [0.001,     0.001] ;
optvarsMax_TASLoop       =   [1.5,       0.5] ;

% Alt loop tunable parameters
optvars_AltLoop          =   {'AP_AltLoop_Kp','AP_AltLoop_Ki'};
optvarsMin_AltLoop       =   [0,         0 ] ;
optvarsMax_AltLoop       =   [1,         0.01 ] ;

% Pwr loop tunable parameters
optvars_PwrLoop          =   {'AP_PwrLoop_Kp'};
optvarsMin_PwrLoop       =   [0] ;
optvarsMax_PwrLoop       =   [2] ;

%% Lateral-Directional Loops
% Roll loop tunable parameters
optvars_RollLoop          =   {'AP_RollLoop_Kp','AP_RollToYawRateLoop_Kp','AP_RollToYawRateLoop_Ki'};  %
optvarsMin_RollLoop       =   [0.05,       0,        0] ;
optvarsMax_RollLoop       =   [3,          1.2,      0.5] ;
 

% GndTrk loop tunable parameters
optvars_GndTrkLoop        =   {'AP_GndTrkLoop_Kp'};  %,'AP_GndTrkLoop_Kd'
optvarsMin_GndTrkLoop     =   [0.001] ;
optvarsMax_GndTrkLoop     =   [0.1 ] ;      

% Yaw Rate loop tunable parameters
% optvars_YawRateLoop          =   {'AP_YawRateLoop_Kp','AP_YawRateLoop_Kd','AP_YawRateLoop_LLFiltNumCoe','AP_YawRateLoop_LLFiltDenCoe','AP_YawRateLoop_LFLLFiltNumCoe','AP_YawRateLoop_LFLLFiltDenCoe'};
optvars_YawRateLoop          =   {'AP_YawRateLoop_Kp'};

optvarsMin_YawRateLoop       =   [0.001] ;
optvarsMax_YawRateLoop       =   [0.2] ;



%HAPS specific flexible Mode Control laws
Opt.FlexControlActive           = 0 ;
Opt.FlexControlInputIndex       = 1 ;             % Elevator Input - Index of the most effective control surface input for flexible mode control - based on the analysis in Module 2:AquilaFltDynAnalysis
Opt.FlexControlOutputIndex      = 16 ;            % Vertical Acceleration Output - Index of the most effective measurement output for the flexible mode control - based on the analysis in Module 2:AquilaFltDynAnalysis

Opt.FlexLoopTuningGoals         = {'Margins'};                                                  %The tuning goals for the Flexible Modes Control Loop

Opt.DesiredFlexLoopGM           = 6;      %in dB
Opt.DesiredFlexLoopPM           = 30;     %in deg

% Flexible Mode(s) control loop tunable parameters
optvars_FlexLoop             =   {'AP_FlexLoop_nzFdbk_Kp','AP_FlexLoop_nzFdbkLLFiltNumCoe','AP_FlexLoop_nzFdbkLLFiltDenCoe' };

optvarsMin_FlexLoop          =   [0.0001,   -1,    -1] ;
optvarsMax_FlexLoop          =   [2,         0,     0 ] ;


%GLA control laws tunable parameters:
Opt.GLAControlActive        =   0 ;

Opt.GLALoopTuningGoals      =   {'Margins'};                                                     %The tuning goals for the GLA Loop

Opt.DesiredGLALoopGM        =   6;      %in dB
Opt.DesiredGLALoopPM        =   30;     %in deg

% GLA control loop tunable parameters
optvars_GLALoop             =   {'AP_GLALoop_nzFdbk_Kp' };

optvarsMin_GLALoop          =   [0.0001] ;
optvarsMax_GLALoop          =   [2 ] ;



%% Initial Conditions
%define the initial condition for the two cases as it would affect the
%convergence for the multi-flight control optimisation
if ( OptimizeMultipleFlts == 1 )
    
    %%Longitudinal Loops 
    %PitchLoop
    optvarsinit_PitchLoop       =   [0.82,    0.34,    0.07];                         %'PitchLoop_Kp','PitchLoop_Ki','PitchLoop_Kq','PitchLoop_qFdbkLLFiltNumCoe','PitchLoop_qFdbkLLFiltDenCoe','PitchLoop_qFdbkLFLLFiltNumCoe','PitchLoop_qFdbkLFLLFiltDenCoe'};
    %ClimbRateLoop
    optvarsinit_ClimbRateLoop   =   [0.05];                                                             %'ClimbRateLoop_Kp','ClimbRateLoop_LLFiltNumCoe','ClimbRateLoop_LLFiltDenCoe'
    %TASLoop
    optvarsinit_TASLoop         =   [0.5,       0];                                             %'TASLoop_Kp','TASLoop_Ki','TASLoop_Kvadv','TASLoop_LLFiltNumCoe','TASLoop_LLFiltDenCoe'
    %AltLoop
    optvarsinit_AltLoop         =   [0.03,     0.0015,     0.93];                                                               %'AltLoop_Kp','AltLoop_Ki','AltLoop_Kd'
    %PwrLoop
    optvarsinit_PwrLoop         =   [0.1,   0.05];                                                    %'PwrLoop_Kp','PwrLoop_Ki','PwrLoop_Kvadv','PwrLoop_LLFiltNumCoe','PwrLoop_LLFiltDenCoe'
    % Lateral/Directional loops
    %RollLoop
    optvarsinit_RollLoop        =   [0.3,      2.0];                                     %'AP_RollLoop_Kp','AP_RollRateLoop_Kp','AP_RollLoop_pFdbkLLFiltNumCoe','AP_RollLoop_pFdbkLLFiltDenCoe','AP_RollToYawRateLoop_Kp','AP_RollToYawRateLoop_Ki'
    %GndTrkLoop
    optvarsinit_GndTrkLoop      =   [0.08];                                                                                     %'AP_GndTrkLoop_Kp','AP_GndTrkLoop_Kd'
    %YawRateLoop
    optvarsinit_YawRateLoop     =   [0.2,   -0.9  -0.9];                                                                       %'AP_YawRateLoop_Kp','AP_YawRateLoop_Kd'

    %Flexible Mode(s) Control loop
    optvarsinit_FlexLoop     =   [0.05,   -0.8975,  -0.6544];                                                                   %'AP_FlexLoop_nzFdbk_Kp','AP_FlexLoop_nzFdbkLLFiltNumCoe','AP_FlexLoop_nzFdbkLLFiltDenCoe'

    % GLA control loop tunable parameters
    optvarsinit_GLALoop      =   [ 0.005 ];                                                                                     %'AP_GLALoop_nzFdbk_Kp'

    
else %OptimizeMultipleFlts ~= TRUE
    
    % Use the optimization results from the previous run as the initial
    % condition for the current run
    if ( Opt.UsePrevOptResultAsIC == 1 )
        %Load the opt results from the previous run - remember that OptOut is the standard output of the optimisation routine (see the
        %OptimiseAquilaClaws_Main.m file)
        load('OptOut');
        
        %%Longitudinal Loops
        %PitchLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'PitchLoop')
            optvarsinit_PitchLoop       =   [OptClaws.TunedConditions.Coeffs.AP_PitchLoop_Kp,    OptClaws.TunedConditions.Coeffs.AP_PitchLoop_Ki,    OptClaws.TunedConditions.Coeffs.AP_PitchLoop_Kq];                         %'PitchLoop_Kp','PitchLoop_Ki','PitchLoop_Kq','PitchLoop_qFdbkLLFiltNumCoe','PitchLoop_qFdbkLLFiltDenCoe','PitchLoop_qFdbkLFLLFiltNumCoe','PitchLoop_qFdbkLFLLFiltDenCoe'};
        end
        %ClimbRateLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'ClimbRateLoop')
            optvarsinit_ClimbRateLoop   =   [OptClaws.TunedConditions.Coeffs.AP_ClimbRateLoop_Kp];                                                   %'ClimbRateLoop_Kp','ClimbRateLoop_Ki','ClimbRateLoop_LLFiltNumCoe','ClimbRateLoop_LLFiltDenCoe'
        end
        %TASLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'TASLoop')
            optvarsinit_TASLoop         =   [OptClaws.TunedConditions.Coeffs.AP_TASLoop_Kp,       OptClaws.TunedConditions.Coeffs.AP_TASLoop_Ki];                                             %'TASLoop_Kp','TASLoop_Ki','TASLoop_Kvadv','TASLoop_LLFiltNumCoe','TASLoop_LLFiltDenCoe'
        end
        %AltLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'AltLoop')
            optvarsinit_AltLoop         =   [OptClaws.TunedConditions.Coeffs.AP_AltLoop_Kp,     OptClaws.TunedConditions.Coeffs.AP_AltLoop_Ki];                                                               %'AltLoop_Kp','AltLoop_Ki','AltLoop_Kd'
        end
        %PwrLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'PwrLoop')
            optvarsinit_PwrLoop         =   [OptClaws.TunedConditions.Coeffs.AP_PwrLoop_Kp];                                                    %'PwrLoop_Kp','PwrLoop_Ki','PwrLoop_Kvadv','PwrLoop_LLFiltNumCoe','PwrLoop_LLFiltDenCoe'
        end
        % The converging IC's for the lat-dir at 9.93m/s
        %RollLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'RollLoop')
            optvarsinit_RollLoop        =   [OptClaws.TunedConditions.Coeffs.AP_RollLoop_Kp,     OptClaws.TunedConditions.Coeffs.AP_RollToYawRateLoop_Kp,  OptClaws.TunedConditions.Coeffs.AP_RollToYawRateLoop_Ki];                              %'AP_RollLoop_Kp','AP_RollRateLoop_Kp','AP_RollLoop_KrollrateAdv','AP_RollLoop_pFdbkLLFiltNumCoe','AP_RollLoop_pFdbkLLFiltDenCoe','AP_RollToYawRateLoop_Kp','AP_RollToYawRateLoop_Ki'
        end
        %GndTrkLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'GndTrkLoop')
            optvarsinit_GndTrkLoop      =   [OptClaws.TunedConditions.Coeffs.AP_GndTrkLoop_Kp];                                                                                    %'AP_GndTrkLoop_Kp','AP_GndTrkLoop_Kd'
        end
        %YawRateLoop
        if isfield(OptClaws.TunedConditions.SLTunerLoops,'YawRateLoop')
            optvarsinit_YawRateLoop     =   [OptClaws.TunedConditions.Coeffs.AP_YawRateLoop_Kp];                                                                      %'AP_YawRateLoop_Kp','AP_YawRateLoop_Kd'
        end
    else  
        
        % Use the default initial conditions - good IC's for the low altitudes
        
         %%Longitudinal Loops 
        %PitchLoop
        optvarsinit_PitchLoop       =   [0.4,      0.1,    0.2];                                               %'PitchLoop_Kp','PitchLoop_Ki','PitchLoop_Kq','PitchLoop_qFdbkLLFiltNumCoe','PitchLoop_qFdbkLLFiltDenCoe','PitchLoop_qFdbkLFLLFiltNumCoe','PitchLoop_qFdbkLFLLFiltDenCoe'}; 
        %ClimbRateLoop
        optvarsinit_ClimbRateLoop   =   [0.2];                                                              %'ClimbRateLoop_Kp','ClimbRateLoop_LLFiltNumCoe','ClimbRateLoop_LLFiltDenCoe'
        %TASLoop
        optvarsinit_TASLoop         =   [1.5,    0.1];                                                         %'TASLoop_Kp','TASLoop_Ki','TASLoop_LLFiltNumCoe','TASLoop_LLFiltDenCoe'
        %AltLoop
        optvarsinit_AltLoop         =   [0.6,     0.01];                                                                          %'AltLoop_Kp','AltLoop_Ki',
        %PwrLoop
        optvarsinit_PwrLoop         =   [0.8];                                                           %'PwrLoop_Kp','PwrLoop_Kvadv','PwrLoop_LLFiltNumCoe','PwrLoop_LLFiltDenCoe'

        %%Lateral - Directional Loops
        %RollLoop
        optvarsinit_RollLoop        =   [1,  0.3, 0.05];                                                                              %'AP_RollLoop_Kp','AP_RollToYawRateLoop_Kp','AP_RollToYawRateLoop_Ki'
        %GndTrkLoop
        optvarsinit_GndTrkLoop      =   [0.05];                                                                                   %'AP_GndTrkLoop_Kp','AP_GndTrkLoop_Kd'
        %YawRateLoop
        optvarsinit_YawRateLoop     =   [0.2];                                                                  %'AP_YawRateLoop_Kp','AP_YawRateLoop_LLFiltNumCoe','AP_YawRateLoop_LLFiltDenCoe'

% %% Good IC's for higher altitude (18km and 9.93)
%          %%Longitudinal Loops 
%         %PitchLoop
%         optvarsinit_PitchLoop       =   [0.4,      0.1,    0.2,     -0.87,   -0.94];                                                 %'PitchLoop_Kp','PitchLoop_Ki','PitchLoop_Kq','PitchLoop_qFdbkLLFiltNumCoe','PitchLoop_qFdbkLLFiltDenCoe','PitchLoop_qFdbkLFLLFiltNumCoe','PitchLoop_qFdbkLFLLFiltDenCoe'}; 
%         %ClimbRateLoop
%         optvarsinit_ClimbRateLoop   =   [0.01,     -0.95,      -0.95];                                                                %'ClimbRateLoop_Kp','ClimbRateLoop_LLFiltNumCoe','ClimbRateLoop_LLFiltDenCoe'
%         %TASLoop
%         optvarsinit_TASLoop         =   [0.1,    0.02,    -0.9,    -0.9];                                                             %'TASLoop_Kp','TASLoop_Ki','TASLoop_LLFiltNumCoe','TASLoop_LLFiltDenCoe'
%         %AltLoop
%         optvarsinit_AltLoop         =   [0.6,     0.025];                                                                             %'AltLoop_Kp','AltLoop_Ki',
%         %PwrLoop
%         optvarsinit_PwrLoop         =   [0.2,     0.5,   -0.95,     -0.95];                                                           %'PwrLoop_Kp','PwrLoop_Kvadv','PwrLoop_LLFiltNumCoe','PwrLoop_LLFiltDenCoe'
% 
%         %%Lateral - Directional Loops
%         %RollLoop
%         optvarsinit_RollLoop        =   [1,  0.3, 0.05];                                                                              %'AP_RollLoop_Kp','AP_RollToYawRateLoop_Kp','AP_RollToYawRateLoop_Ki'
%         %GndTrkLoop
%         optvarsinit_GndTrkLoop      =   [0.05];                                                                                       %'AP_GndTrkLoop_Kp','AP_GndTrkLoop_Kd'
%         %YawRateLoop
%         optvarsinit_YawRateLoop     =   [0.2,   -0.95,  -0.95];                                                                       %'AP_YawRateLoop_Kp','AP_YawRateLoop_LLFiltNumCoe','AP_YawRateLoop_LLFiltDenCoe'
% 
%         
%         %%Flexible Mode(s) Control Loop
%         optvarsinit_FlexLoop        =   [0.05,   -0.8975,  -0.6544];                                                                  %'AP_FlexLoop_nzFdbk_Kp','AP_FlexLoop_nzFdbkLLFiltNumCoe','AP_FlexLoop_nzFdbkLLFiltDenCoe'
%     
%         %%GLA Control Loop
%         optvarsinit_GLALoop         =   [ 0.005 ];                                                                                     %'AP_GLALoop_nzFdbk_Kp'

    end
   
end

Opt.optvars      = {} ;
Opt.optvarsinit  = [] ;
Opt.optvarsMin   = [] ;
Opt.optvarsMax   = [] ;

if (contains(OptConfig,'PitchLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_PitchLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_PitchLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_PitchLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_PitchLoop] ;
end

if (contains(OptConfig,'ClimbRateLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_ClimbRateLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_ClimbRateLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_ClimbRateLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_ClimbRateLoop] ;
end

if (contains(OptConfig,'TASLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_TASLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_TASLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_TASLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_TASLoop] ;
end

if (contains(OptConfig,'AltLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_AltLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_AltLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_AltLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_AltLoop] ;
end

if (contains(OptConfig,'PwrLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_PwrLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_PwrLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_PwrLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_PwrLoop] ;
end
  

if (contains(OptConfig,'RollLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_RollLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_RollLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_RollLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_RollLoop] ;
end

if (contains(OptConfig,'GndTrkLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_GndTrkLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_GndTrkLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_GndTrkLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_GndTrkLoop] ;
end

if (contains(OptConfig,'YawRateLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_YawRateLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_YawRateLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_YawRateLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_YawRateLoop] ;
end

if (contains(OptConfig,'FlexLoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_FlexLoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_FlexLoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_FlexLoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_FlexLoop] ;
end

if (contains(OptConfig,'GLALoop'))
    
    Opt.optvars          =   [Opt.optvars, optvars_GLALoop];
    Opt.optvarsinit      =   [Opt.optvarsinit, optvarsinit_GLALoop];
    Opt.optvarsMin       =   [Opt.optvarsMin, optvarsMin_GLALoop] ;
    Opt.optvarsMax       =   [Opt.optvarsMax, optvarsMax_GLALoop] ;
end

if(~(contains(OptConfig,'PitchLoop') || contains(OptConfig,'ClimbRateLoop') || contains(OptConfig,'TASLoop') || ...
     contains(OptConfig,'AltLoop') || contains(OptConfig,'PwrLoop') || contains(OptConfig,'RollLoop') || contains(OptConfig,'GndTrkLoop') || ...
     contains(OptConfig,'YawRateLoop') || contains(OptConfig,'FlexLoop') || contains(OptConfig,'GLALoop') ) )
   
	error('Make sure the configuration exists in the Aquila Claws Simulink model');

end


Opt.OptConfig                   = OptConfig ;
     

    

    
