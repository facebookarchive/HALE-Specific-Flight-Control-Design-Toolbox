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
% Configure the objective array for the Pitch loop

PitchLoopTuningGoals = cellarray2str(optOptions.PitchLoopTuningGoals) ;


% Frequency-Loop shaping requirements
% -20dB/decade within the bandwidth and then -40dB/decade beyond it;
% Freq_n  = optOptions.DesiredPitchLoopBW.*[(10^-1), (10^0), 2*(10^0), (10^1)] ;
% Gains_n = [10, 1 , 0.5, -0.01];    %20dB/decase

Freq_n  = optOptions.DesiredPitchLoopBW.*[(10^-1), (10^0), (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase


PitchLoopFreqShaping            =   TuningGoal.LoopShape('theta_meas',frd(Gains_n,Freq_n)); % loop transfer measured at theta_Deg (G*K)theta_Deg
PitchLoopFreqShaping.Focus      =   [10^-2, (4*10^1)];
PitchLoopFreqShaping.Name       =   'Pitch/rate Inner Loop';
PitchLoopFreqShaping.Openings   =   {'theta_cmd_uncapped_deg','climb_rate_cmd_altloop_mps'} ;  %'thrCmd_Norm'

% PitchLoopFreqShaping.Stabilize  =   0 ;
% looptransfer(T_q_InnerLoop,'elv',-1,'theta');  %Break the outer loop in
% theta, this would give the open loop loop gain (G*K)

% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 20% mismatch with reference model 1/(thau*s+1)
PitchLoopTrackReq               = TuningGoal.StepTracking({'theta_ref'},{'theta_meas'},1.15/optOptions.DesiredPitchLoopBW);   %1/DesiredThetaLoop_Bandwidth is the time constant of the reference model
PitchLoopTrackReq.Openings      = {'theta_cmd_uncapped_deg','climb_rate_cmd_altloop_mps'} ;  %'thrCmd_Norm'
PitchLoopTrackReq.RelGap        = 0.2;    %looptransfer(T_q_InnerLoop,'elv',-1,'theta');  %Break the outer loop in

% PitchLoop.TrackReq=PitchLoopTrackReq;
% theta, this would give the open loop loop gain


% Margins Requirement
% Gain and phase margins at plant inputs and outputs
if(optOptions.FlexControlActive == true )
   PitchLoopMarginReq1             = TuningGoal.Margins('elvCmdDeg_PitchLoop',optOptions.DesiredPitchLoopGM,optOptions.DesiredPitchLoopPM);  %When Flexible modes control loops are added
else
    PitchLoopMarginReq1             = TuningGoal.Margins('elvCmd_Deg',optOptions.DesiredPitchLoopGM,optOptions.DesiredPitchLoopPM);
end
PitchLoopMarginReq1.Focus       = [10^-3, 4*10^1];
PitchLoopMarginReq1.Openings    = {};     %{'theta_cmd_uncapped_deg','climb_rate_cmd_altloop_mps'}  ;


PitchLoopMarginReq2             = TuningGoal.Margins('q_meas',optOptions.DesiredPitchLoopGM,optOptions.DesiredPitchLoopPM);
PitchLoopMarginReq2.Focus       = [10^-3, 4*10^1];
PitchLoopMarginReq2.Openings    = {};     %{'theta_cmd_uncapped_deg'}  ;


% Overshoot Requirement
PitchLoopOvershootReq           = TuningGoal.Overshoot({'theta_ref'},{'theta_Deg'},optOptions.DesiredPitchLoopOV);    %Desired Overshoot of the pitch loop (from theta_ref to theta)
PitchLoopOvershootReq.Openings  = {'theta_cmd_uncapped_deg','thrCmd_Norm'} ;


% MaxFreq Requirement
% Fast dynamics: The magnitude of the closed-loop poles must not exceed 25 to
% prevent fast dynamics and jerky transients
PitchLoopPoleReq            = TuningGoal.Poles(0,0,optOptions.DesiredPitchLoopMaxFreq);
PitchLoopPoleReq.Openings   = {'theta_cmd_uncapped_deg','thrCmd_Norm'}  ;


% Disturbance Rejection Requirement
% Input Disturbance rejection: 
PitchLoopAttenuationShape               = frd([10 10 1 1],[0 optOptions.DesiredPitchLoopBW 10*optOptions.DesiredPitchLoopBW 100*optOptions.DesiredPitchLoopBW]);      
PitchLoopDisturbnceRejectReq            = TuningGoal.Rejection('w_d',PitchLoopAttenuationShape ) ;
PitchLoopDisturbnceRejectReq.Openings   = {'theta_cmd_uncapped_deg','thrCmd_Norm'}  ;


% Sensitivity Requirement
% Output Disturbance rejection (Sensitivity Requirements): frd([0.01 1 1],[0.001 0.1 100])
PitchLoopMaxSensShape            = frd([0.001 1 1],[0 optOptions.DesiredPitchLoopBW 10*optOptions.DesiredPitchLoopBW]);      
PitchLoopSensitivityReq          = TuningGoal.Sensitivity('theta_meas',PitchLoopMaxSensShape) ;
PitchLoopSensitivityReq.Focus    = [0.1*optOptions.DesiredPitchLoopBW, 10*optOptions.DesiredPitchLoopBW];
PitchLoopSensitivityReq.Openings = {'theta_cmd_uncapped_deg','thrCmd_Norm'} ;% to calculate the sensitvity function broken at the measure output signal, leave all the loops closed



% Freq_Tracking Requirement
%  Relative error requirements:
%  Track the setpoint with a 1/optOptions.DesiredPitchLoopBW response time, less than 3% 
%  steady-state error, and less than 20% peak error.

% The frequency tracking requirement,  should be in-line with the FreqShaping requirement:
PitchLoopresponse_time          = 0.35/(optOptions.DesiredPitchLoopBW/(2*pi)) ;
PitchLoopFreqTrackReq           = TuningGoal.Tracking({'theta_ref'},{'theta_Deg'},(2*PitchLoopresponse_time), 0.02, 1.3);   %1/DesiredThetaLoop_Bandwidth is the time constant of the reference model
PitchLoopFreqTrackReq.Openings  = {'theta_cmd_uncapped_deg','thrCmd_Norm'} ;


% Closed Loop Poles Requirement - The following requirements should be transferred to the OptimizeAquilaClaws_Discrete_GS.m script
PitchLoopMinDecay           = 1e-9;
PitchLoopMinDamping         = 0.1;
PitchLoopMaxFrequency       = 100;
PitchLoopClosedPolesReq     = TuningGoal.Poles(PitchLoopMinDecay);    


PitchLoop_TuningGoals_list=[PitchLoopFreqShaping, PitchLoopTrackReq, PitchLoopMarginReq1, PitchLoopMarginReq2, PitchLoopOvershootReq, PitchLoopPoleReq, PitchLoopDisturbnceRejectReq, PitchLoopSensitivityReq, PitchLoopFreqTrackReq, PitchLoopClosedPolesReq];

Objectives_PitchLoop = [];

if( contains(PitchLoopTuningGoals,'Freq_Shaping') )

    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopFreqShaping ] ;
end

if( contains(PitchLoopTuningGoals,'Step_Tracking') )

    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopTrackReq ] ;
end

if( contains(PitchLoopTuningGoals,'Margins') )

%     Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopMarginReq1, PitchLoopMarginReq2 ] ;
    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopMarginReq1 ] ;

end

if( contains(PitchLoopTuningGoals,'Overshoot') )

    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopOvershootReq ] ;
end

if( contains(PitchLoopTuningGoals,'MaxFreq') )
    
    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopPoleReq ] ;
end

if( contains(PitchLoopTuningGoals,'DisturbanceReject') )   %Input disturbance rejection requirements
    
    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopDisturbnceRejectReq ] ;
end

if( contains(PitchLoopTuningGoals,'Sensitivity') )         %Sensitivity requirement 
    
    Objectives_PitchLoop    =   [ Objectives_PitchLoop, PitchLoopSensitivityReq ] ;
end

if( contains(PitchLoopTuningGoals,'Freq_Tracking') )

    % viewSpec(FreqTrackReq);
    Objectives_PitchLoop  =   [ Objectives_PitchLoop, PitchLoopFreqTrackReq ] ;
end

if( contains(PitchLoopTuningGoals,'ClosedLoopPoles') )

    viewSpec(ClosedPolesReq);
    Objectives_PitchLoop    =   [ Objectives_PitchLoop, PitchLoopClosedPolesReq ] ;
end

PitchLoop=struct;
PitchLoop.Name='PitchLoop';
PitchLoop.TuningGoals_list=PitchLoop_TuningGoals_list;
PitchLoop.Objectives=Objectives_PitchLoop;
