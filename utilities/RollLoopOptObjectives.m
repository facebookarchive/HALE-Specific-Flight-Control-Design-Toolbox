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
%   Configure the objective array for the Roll loop


RollLoopTuningGoals = cellarray2str(optOptions.RollLoopTuningGoals) ;


% Frequency-Loop shaping requirements
% -20dB/decade within the bandwidth and then -40dB/decade beyond it;
% Freq_n  = optOptions.DesiredRollLoopBW.*[(10^-1), (10^0), 2*(10^0), (10^1)] ;
% Gains_n = [10, 1 , 0.5, -0.01];    %20dB/decase

% Freq_n_phi  = optOptions.DesiredRollLoopBW.*[(10^-2), (10^-1), (10^0), (10^1)] ;
% Gains_n_phi = [-0.1, -0.1 , -0.1 , -0.01];    %20dB/decase

Freq_n_phi  = optOptions.DesiredRollLoopBW.*[(10^-1), (10^0), (10^1)] ;
Gains_n_phi = [10, 1 , -0.1];    %20dB/decase


RollLoopFreqShaping            =   TuningGoal.LoopShape('phi_meas_deg',frd(Gains_n_phi,Freq_n_phi)); % loop transfer measured at phi_meas_deg (G*K)
RollLoopFreqShaping.Focus      =   [10^-2, (10^1)];
RollLoopFreqShaping.Name       =   'Roll/rate Intermediate Loop';
RollLoopFreqShaping.Openings   =   {'phi_cmd_deg'} ;

% Step-Tracking requirements    
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 30% mismatch with reference model 1/(thau*s+1)
RollLoopTrackReq               =    TuningGoal.StepTracking({'phi_ref'},{'phi_meas_deg'},1.15/optOptions.DesiredRollLoopBW);   %1/DesiredThetaLoop_Bandwidth is the time constant of the reference model
RollLoopTrackReq.Name          =    'Roll/rate Intermediate Loop';
RollLoopTrackReq.Openings      =    {'phi_cmd_deg'} ;                  %{'phi_cmd_deg','diffThrCmd_Norm'} ;
RollLoopTrackReq.RelGap        =    0.2;


% Frequency-Loop shaping requirements
% Gain and phase margins at plant inputs and outputs
RollLoopMarginReq1             = TuningGoal.Margins('ailCmd_Deg',optOptions.DesiredRollLoopGM,optOptions.DesiredRollLoopPM);
RollLoopMarginReq1.Focus       = [10^-2, 4*10^1];
RollLoopMarginReq1.Name        = 'Roll/rate Intermediate Loop - Margin measured at the loop input (ailCmd_Deg)';
RollLoopMarginReq1.Openings    = {'phi_cmd_deg'}  ; 


RollLoopMarginReq2             = TuningGoal.Margins('phi_meas_deg',optOptions.DesiredRollLoopGM,optOptions.DesiredRollLoopPM);
RollLoopMarginReq2.Focus       = [10^-2, 4*10^1];
RollLoopMarginReq2.Name        = 'Roll/rate Intermediate Loop - Margin measured at the loop output (phi_meas_deg)';
RollLoopMarginReq2.Openings    = {'phi_cmd_deg'}  ;

RollLoopMarginReq3             = TuningGoal.Margins('rollRate_meas_dps',optOptions.DesiredRollLoopGM,optOptions.DesiredRollLoopPM);
RollLoopMarginReq3.Focus       = [10^-2, 4*10^1];
RollLoopMarginReq3.Name        = 'Roll/rate Intermediate Loop - @ roll rate output';
RollLoopMarginReq3.Openings    = {'phi_cmd_deg'}  ;


% Overshoot Requirement
RollLoopOvershootReq           = TuningGoal.Overshoot({'phi_ref'},{'phi_meas_deg'},optOptions.DesiredRollLoopOV);    %Desired Overshoot of the Roll loop (from roll_ref to roll_meas_deg)
RollLoopOvershootReq.Openings  = {'phi_cmd_deg'} ;


% MaxFreq requirement
% Fast dynamics: The magnitude of the closed-loop poles must not exceed 25 to
% prevent fast dynamics and jerky transients
RollLoopPoleReq            = TuningGoal.Poles(0,0,optOptions.DesiredRollLoopMaxFreq);
RollLoopPoleReq.Openings   = {'phi_cmd_deg'}  ;


% Disturbance Rejection Requirement
% Input Disturbance rejection: 
RollLoopAttenuationShape               = frd([10 10 1 1],[0 optOptions.DesiredRollLoopBW 10*optOptions.DesiredRollLoopBW 100*optOptions.DesiredRollLoopBW]);      
RollLoopDisturbnceRejectReq            = TuningGoal.Rejection('in_dis_ail',RollLoopAttenuationShape) ;
RollLoopDisturbnceRejectReq.Openings   = {'phi_cmd_deg'}  ;


% Sensitivity Requirement
% Output Disturbance rejection (Sensitivity Requirements): frd([0.01 1 1],[0.001 0.1 100])
RollLoopMaxSensShape            = frd([0.001 1 1],[0 optOptions.DesiredRollLoopBW 10*optOptions.DesiredRollLoopBW]);      
RollLoopSensitivityReq          = TuningGoal.Sensitivity('phi_meas_deg',RollLoopMaxSensShape) ;
RollLoopSensitivityReq.Focus    = [0.1*optOptions.DesiredRollLoopBW, 10*optOptions.DesiredRollLoopBW];
RollLoopSensitivityReq.Openings = {'phi_cmd_deg'} ;

% FreqTracking Requirement
%  Relative error requirements:
%  Track the setpoint with a 1/optOptions.DesiredRollLoopBW response time, less than 3% 
%   steady-state error, and less than 20% peak error.

% The frequency tracking requirement,  should be in-line with the FreqShaping requirement:
RollLoopresponse_time          = 0.35/(optOptions.DesiredRollLoopBW/(2*pi)) ;
RollLoopFreqTrackReq           = TuningGoal.Tracking({'phi_ref'},{'phi_meas_deg'},(2*RollLoopresponse_time), 0.02, 1.3);   %1/DesiredRollLoop_Bandwidth is the time constant of the reference model
RollLoopFreqTrackReq.Openings  = {'phi_cmd_deg'} ;


% Closed Loop Poles Requirement
RollLoopMinDecay = 1e-9;
RollLoopMinDamping = 0.1;
RollLoopMaxFrequency = 100;
RollLoopClosedPolesReq             = TuningGoal.Poles(RollLoopMinDecay);    


RollLoop_TuningGoals_list=[RollLoopFreqShaping, RollLoopTrackReq, RollLoopMarginReq1, RollLoopMarginReq3, RollLoopOvershootReq, RollLoopPoleReq, RollLoopDisturbnceRejectReq, RollLoopSensitivityReq, RollLoopFreqTrackReq, RollLoopClosedPolesReq];

Objectives_RollLoop = [];

if( contains(RollLoopTuningGoals,'Freq_Shaping') )

    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopFreqShaping ] ;
end

if( contains(RollLoopTuningGoals,'Step_Tracking') )

    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopTrackReq ] ;
end

if( contains(RollLoopTuningGoals,'Margins') )

    %Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopMarginReq1, RollLoopMarginReq2, RollLoopMarginReq3 ] ;
    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopMarginReq1, RollLoopMarginReq2 ] ;
end

if( contains(RollLoopTuningGoals,'Overshoot') )

    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopOvershootReq ] ;
end

if( contains(RollLoopTuningGoals,'MaxFreq') )
    
    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopPoleReq ] ;
end

if( contains(RollLoopTuningGoals,'DisturbanceReject') )   %Input disturbance rejection requirements
    
    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopDisturbnceRejectReq ] ;
end

if( contains(RollLoopTuningGoals,'Sensitivity') )         %Sensitivity requirement
    
    Objectives_RollLoop    =   [ Objectives_RollLoop, RollLoopSensitivityReq ] ;
end

if( contains(RollLoopTuningGoals,'Freq_Tracking') )

    Objectives_RollLoop  =   [ Objectives_RollLoop, RollLoopFreqTrackReq ] ;
end

if( contains(RollLoopTuningGoals,'ClosedLoopPoles') )

    viewSpec(ClosedPolesReq);
    Objectives_RollLoop    =   [ Objectives_RollLoop, RollLoopClosedPolesReq ] ;
end


RollLoop=struct;
RollLoop.Name='RollLoop';
RollLoop.TuningGoals_list=RollLoop_TuningGoals_list;
RollLoop.Objectives=Objectives_RollLoop;
