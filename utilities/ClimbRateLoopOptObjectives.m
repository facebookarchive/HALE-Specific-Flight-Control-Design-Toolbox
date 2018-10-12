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


% Configure the objective array for the Climb Rate loop

ClimbRateLoopTuningGoals = cellarray2str(optOptions.ClimbRateLoopTuningGoals) ;

% Frequency-Loop shaping requirements

Freq_n  = optOptions.DesiredClimbRateLoopBW.*[(10^-1), 1, (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase

ClimbRateLoopFreqShaping            =   TuningGoal.LoopShape('climb_rate_mps',frd(Gains_n,Freq_n)); % loop transfer measured at climb_rate_mps
ClimbRateLoopFreqShaping.Focus      =   [10^-3, (4*10^1)];
ClimbRateLoopFreqShaping.Openings   =   {'climb_rate_cmd_spdloop_mps','thrCmd_Norm','elvCmd_flex_deg','splCmd_Deg'} ;
ClimbRateLoopFreqShaping.Name       =   'Climb Rate Intermediate Loop';



% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 50% mismatch with reference model 1/(thau*s+1)
ClimbRateLoopTrackReq = TuningGoal.StepTracking({'climb_rate_cmd_spdloop_mps'},{'climb_rate_meas_mps'},(1.15/optOptions.DesiredClimbRateLoopBW));   %1/optOptions.DesiredClimbRateLoopBW is the time constant of the reference model
ClimbRateLoopTrackReq.Openings      =   {'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm','elvCmd_flex_deg'} ;  % Open the throttle loop so the climb rate command in the spd loop could converge, otherwise, throttle loop would reduce the climb rate loop DC gain in the speed loop

ClimbRateLoopTrackReq.RelGap        =   0.5;  


% Margins requirements
% Gain and phase margins at plant inputs and outputs
ClimbRateLoopMarginReq1             = TuningGoal.Margins('theta_cmd_uncapped_deg',optOptions.DesiredClimbRateLoopGM,optOptions.DesiredClimbRateLoopPM);
ClimbRateLoopMarginReq1.Openings    =  {};         %{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'} ;

ClimbRateLoopMarginReq1.Focus       = [10^-3, 4*10^1];
ClimbRateLoopMarginReq1.Name        =  'Climb Rate Intermediate Loop - Margin at the loop input (theta_cmd_uncapped_deg)';


% Sensitivity at the ouput measurements
ClimbRateLoopMarginReq2             = TuningGoal.Margins('climb_rate_meas_mps',optOptions.DesiredClimbRateLoopGM,optOptions.DesiredClimbRateLoopPM);
ClimbRateLoopMarginReq2.Openings    =  {};          %{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'} ;

ClimbRateLoopMarginReq2.Focus       = [10^-3, 4*10^1];
ClimbRateLoopMarginReq2.Name        = 'Climb Rate Intermediate Loop - Margin at the loop output (climb_rate_meas_mps)';


ClimbRateLoop_TuningGoals_list=[ClimbRateLoopFreqShaping, ClimbRateLoopTrackReq, ClimbRateLoopMarginReq1, ClimbRateLoopMarginReq2];


Objectives_ClimbRateLoop = [];

if( contains(ClimbRateLoopTuningGoals,'Freq_Shaping') )
    
    Objectives_ClimbRateLoop  =   [ Objectives_ClimbRateLoop, ClimbRateLoopFreqShaping ] ;
end

if( contains(ClimbRateLoopTuningGoals,'Step_Tracking') )  

    Objectives_ClimbRateLoop  =   [ Objectives_ClimbRateLoop, ClimbRateLoopTrackReq ] ;
end

if( contains(ClimbRateLoopTuningGoals,'Margins') )

   
%     Objectives_ClimbRateLoop  =   [ Objectives_ClimbRateLoop, ClimbRateLoopMarginReq1, ClimbRateLoopMarginReq2 ] ;
    Objectives_ClimbRateLoop  =   [ Objectives_ClimbRateLoop, ClimbRateLoopMarginReq2 ] ;

end

%% Add hard Objectives regarding the gains constrainst
% ClimbRateLoopLLFiltDenCoeGainReq        = TuningGoal.Gain('climbRateLoop_LLFiltDenCoe_in','climbRateLoop_LLFiltDenCoe_out',1e-10);
% ClimbRateLoopLLFiltDenCoeGainReq.Name   = 'Hard constraint on the climb Rate loop filter den coefficient'; 
% Objectives_ClimbRateLoop                = [ Objectives_ClimbRateLoop, ClimbRateLoopLLFiltDenCoeGainReq ] ;

%ClimbRateLoop=MakeLoopStruct(ClimbRateLoop_TuningGoals_list,Objectives_ClimbRateLoop,'ClimbRateLoop',-1);
ClimbRateLoop=struct;
ClimbRateLoop.Name='ClimbRateLoop';
ClimbRateLoop.TuningGoals_list=ClimbRateLoop_TuningGoals_list;
ClimbRateLoop.Objectives=Objectives_ClimbRateLoop;