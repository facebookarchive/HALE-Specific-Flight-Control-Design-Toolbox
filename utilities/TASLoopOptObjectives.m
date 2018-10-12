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
%  Configure the objective array for the TAS loop

TASLoopTuningGoals = cellarray2str(optOptions.TASLoopTuningGoals) ;


% Frequency-Loop shaping requirements

Freq_n  = optOptions.DesiredTASLoopBW.*[(10^-1), 1, (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase

TASLoopFreqShaping              =   TuningGoal.LoopShape('tas_mps',frd(Gains_n,Freq_n)); % loop transfer measured at tas_mps ( This gives the G*K)
TASLoopFreqShaping.Focus        =   [10^-3, (4*10^1)];
TASLoopFreqShaping.Name         =   'TAS_Loop' ;
TASLoopFreqShaping.Openings     =   {'splCmd_Deg'};      %{'thrCmd_Norm'} ;
%     TASLoopFreqShaping.Stabilize    =   0;


% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 20% mismatch with reference model 1/(thau*s+1)
TASLoopTrackReq             =   TuningGoal.StepTracking({'tas_cmd_mps'},{'tas_mps'},1.0/optOptions.DesiredTASLoopBW);   %1/DesiredThetaLoop_Bandwidth is the time constant of the reference model
TASLoopTrackReq.Openings    =   {'splCmd_Deg'};        %{'thrCmd_Norm'} ;
TASLoopTrackReq.RelGap      =   0.2;


% Margins Requirement
%Frequency-Loop shaping requirements
% Gain and phase margins at plant inputs and outputs
TASLoopMarginReq1           =   TuningGoal.Margins('climb_rate_cmd_spdloop_uncapped_mps',optOptions.DesiredTASLoopGM,optOptions.DesiredTASLoopPM);
TASLoopMarginReq1.Openings  =   {};           %{'thrCmd_Norm'} ;
TASLoopMarginReq1.Focus     =   [10^-3, 4*10^1];
TASLoopMarginReq1.Name      =   'TAS Loop Outer Loop - Margin measured at the loop input (climb_rate_cmd)';

TASLoopMarginReq2           =	TuningGoal.Margins('tas_meas_mps',optOptions.DesiredTASLoopGM,optOptions.DesiredTASLoopPM);
TASLoopMarginReq2.Openings  =	{};           %{'thrCmd_Norm'} ;
TASLoopMarginReq2.Focus     =   [10^-3, 4*10^1];
TASLoopMarginReq2.Name      =   'TAS Loop Outer Loop - Margin measured at the loop output (tas_meas_mps)';


% Overshoot Requirement
TASLoopOvershootReq             =   TuningGoal.Overshoot({'tas_cmd_mps'},{'tas_mps'},optOptions.DesiredTASLoopOV);    %Desired Overshoot of the TAS loop (from tas command to tas)
TASLoopOvershootReq.Openings    =   {};     %{'thrCmd_Norm'} ;


TASLoop_TuningGoals_list=[TASLoopFreqShaping, TASLoopTrackReq, TASLoopMarginReq1, TASLoopMarginReq2, TASLoopOvershootReq];


Objectives_TASLoop = [];

if( contains(TASLoopTuningGoals,'Freq_Shaping') )

    Objectives_TASLoop  =   [ Objectives_TASLoop, TASLoopFreqShaping ] ;
end

if( contains(TASLoopTuningGoals,'Step_Tracking') )   
    
    Objectives_TASLoop  =   [ Objectives_TASLoop, TASLoopTrackReq ] ;
end

if( contains(TASLoopTuningGoals,'Margins') )
    
    Objectives_TASLoop  =   [ Objectives_TASLoop, TASLoopMarginReq1, TASLoopMarginReq2 ] ;
%     Objectives_TASLoop  =   [ Objectives_TASLoop, TASLoopMarginReq1 ] ;

end

if( contains(TASLoopTuningGoals,'Overshoot') )    
    
    Objectives_TASLoop  =   [ Objectives_TASLoop, TASLoopOvershootReq ] ;
end

TASLoop=struct;
TASLoop.Name='TASLoop';
TASLoop.TuningGoals_list=TASLoop_TuningGoals_list;
TASLoop.Objectives=Objectives_TASLoop;
