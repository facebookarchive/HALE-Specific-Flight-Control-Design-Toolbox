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
% Configure the objective array for the Pwr loop loop

PwrLoopTuningGoals = cellarray2str(optOptions.PwrLoopTuningGoals) ;


%Frequency-Loop shaping requirements

Freq_n  = optOptions.DesiredPwrLoopBW.*[(10^-1), 1, (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase
%     PwrLoopFreqShaping            =   TuningGoal.LoopShape('thrCmd_Norm',frd(Gains_n,Freq_n)); % loop transfer measured at thrCmd_Norm
PwrLoopFreqShaping            =   TuningGoal.LoopShape('climb_rate_meas_mps',frd(Gains_n,Freq_n)); % loop transfer measured at climb_rate_mps or climb_rate_meas_mps

PwrLoopFreqShaping.Focus      =   [10^-3, (4*10^1)];
PwrLoopFreqShaping.Openings   =   {'climb_rate_cmd_altloop_mps','theta_cmd_uncapped_deg'}; %the 'elvCmd_Deg' or the TAS loop must be opened as well, otherwise, the TAS loop and the Alt loop would fight.
PwrLoopFreqShaping.Name       =   'Pwr Loop Inner Loop';


% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 50% mismatch with reference model 1/(thau*s+1)
PwrLoopTrackReq = TuningGoal.StepTracking({'climb_rate_cmd_altloop_mps'},{'climb_rate_meas_mps'},(1.1/optOptions.DesiredPwrLoopBW));   %1/optOptions.DesiredPwrLoopBW is the time constant of the reference model
PwrLoopTrackReq.Openings      =   {'climb_rate_cmd_altloop_mps', 'theta_cmd_uncapped_deg'} ;  %the 'elvCmd_Deg' or the TAS loop must be opened as well, otherwise, the TAS loop and the Alt loop would fight. 
PwrLoopTrackReq.RelGap        =   0.7;  
PwrLoopTrackReq.Name          =   'Pwr Loop Inner Loop - Step Tracking';

% Margins Requirement
%Frequency-Loop shaping requirements
% Gain and phase margins at plant inputs and outputs
PwrLoopMarginReq1             = TuningGoal.Margins('thrCmd_Norm',optOptions.DesiredPwrLoopGM,optOptions.DesiredPwrLoopPM);
PwrLoopMarginReq1.Openings    =  {};            %'climb_rate_cmd_altloop_mps','theta_cmd_uncapped_deg'} ;%Shouldn't open it
PwrLoopMarginReq1.Focus       = [10^-3, 4*10^1];
PwrLoopMarginReq1.Name        =  'Pwr Loop - Margin at the loop input command (thrCmd_Norm)';

% Sensitivity at the ouput measurements
PwrLoopMarginReq2             = TuningGoal.Margins('climb_rate_meas_mps',optOptions.DesiredPwrLoopGM,optOptions.DesiredPwrLoopPM);
PwrLoopMarginReq2.Openings    =  {};            %'climb_rate_cmd_altloop_mps','theta_cmd_uncapped_deg'} ;%Shouldn't open it
PwrLoopMarginReq2.Focus       = [10^-3, 4*10^1];
PwrLoopMarginReq2.Name        =  'Pwr Loop - Margin at the loop output (climb_rate_meas_mps)';



PwrLoop_TuningGoals_list=[PwrLoopFreqShaping, PwrLoopTrackReq, PwrLoopMarginReq1, PwrLoopMarginReq2];



Objectives_PwrLoop = [];

if( contains(PwrLoopTuningGoals,'Freq_Shaping') )
    
    Objectives_PwrLoop  =   [ Objectives_PwrLoop, PwrLoopFreqShaping ] ;
end

if( contains(PwrLoopTuningGoals,'Step_Tracking') )  

    Objectives_PwrLoop  =   [ Objectives_PwrLoop, PwrLoopTrackReq ] ;
end

if( contains(PwrLoopTuningGoals,'Margins') )
   
    Objectives_PwrLoop  =   [ Objectives_PwrLoop, PwrLoopMarginReq1, PwrLoopMarginReq2 ] ;
%     Objectives_PwrLoop  =   [ Objectives_PwrLoop, PwrLoopMarginReq2 ] ;

end

PwrLoop=struct;
PwrLoop.Name='PwrLoop';
PwrLoop.TuningGoals_list=PwrLoop_TuningGoals_list;
PwrLoop.Objectives=Objectives_PwrLoop;
