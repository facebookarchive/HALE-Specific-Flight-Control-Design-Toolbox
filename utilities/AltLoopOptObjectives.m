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
% 

AltLoopTuningGoals = cellarray2str(optOptions.AltLoopTuningGoals) ;


% Frequency-Loop shaping requirements

Freq_n  = optOptions.DesiredAltLoopBW.*[(10^-1), 1, (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase

% AltLoopFreqShaping            =   TuningGoal.LoopShape('climb_rate_cmd_altloop_mps',frd(Gains_n,Freq_n)); % loop transfer measured at climb_rate_cmd_altloop_mps
AltLoopFreqShaping            =   TuningGoal.LoopShape('presAlt_meas_m',frd(Gains_n,Freq_n)); % loop transfer measured at presAlt_meas_m
AltLoopFreqShaping.Openings   =   {'splCmd_Deg'};        %{'thrCmd_Norm'} ;
AltLoopFreqShaping.Focus      =   [10^-3, (4*10^1)];
AltLoopFreqShaping.Name       =   'Alt Loop Outer Loop';


% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 50% mismatch with reference model 1/(thau*s+1)
AltLoopTrackReq = TuningGoal.StepTracking({'alt_cmd_m'},{'presAlt_meas_m'},(1.15/optOptions.DesiredAltLoopBW));   %1/optOptions.DesiredAltLoopBW is the time constant of the reference model
AltLoopTrackReq.Openings      =   {'splCmd_Deg'};
AltLoopTrackReq.RelGap        =   0.2; 


%Margins Requirement
%Frequency-Loop shaping requirements
% Gain and phase margins at plant inputs and outputs
AltLoopMarginReq1             = TuningGoal.Margins('climb_rate_cmd_altloop_mps',optOptions.DesiredAltLoopGM,optOptions.DesiredAltLoopPM);
AltLoopMarginReq1.Focus       = [10^-3, 4*10^1];

% Sensitivity at the ouput measurements
AltLoopMarginReq2             = TuningGoal.Margins('presAlt_meas_m',optOptions.DesiredAltLoopGM,optOptions.DesiredAltLoopPM);
AltLoopMarginReq2.Focus       = [10^-3, 4*10^1];
AltLoopMarginReq2.Name        =  'Altitude Loop Outer Loop - Margin measured at the loop output (presAlt_meas_m)';



AltLoop_TuningGoals_list=[AltLoopFreqShaping, AltLoopTrackReq, AltLoopMarginReq1, AltLoopMarginReq2];


Objectives_AltLoop = [];

if( contains(AltLoopTuningGoals,'Freq_Shaping') )

    Objectives_AltLoop  =   [ Objectives_AltLoop, AltLoopFreqShaping ] ;
end

if( contains(AltLoopTuningGoals,'Step_Tracking') )  

    Objectives_AltLoop  =   [ Objectives_AltLoop, AltLoopTrackReq ] ;
end

if( contains(AltLoopTuningGoals,'Margins') )
   
%     Objectives_AltLoop  =   [ Objectives_AltLoop, AltLoopMarginReq1, AltLoopMarginReq2 ] ;
    Objectives_AltLoop  =   [ Objectives_AltLoop, AltLoopMarginReq2 ] ;

end

%AltLoop=MakeLoopStruct(AltLoop_TuningGoals_list,Objectives_AltLoop,'AltLoop',-1);
AltLoop=struct;
AltLoop.Name='AltLoop';
AltLoop.TuningGoals_list=AltLoop_TuningGoals_list;
AltLoop.Objectives=Objectives_AltLoop;