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


% Configure the objective array for the GndTrk loop

GndTrkLoopTuningGoals = cellarray2str(optOptions.GndTrkLoopTuningGoals) ;


%Frequency-Loop shaping requirements
%     -20dB/decade within the bandwidth and then -40dB/decade beyond it;
%     Freq_n  = optOptions.DesiredGndTrkLoopBW.*[(10^-1), (10^0), 2*(10^0), (10^1)] ;
%     Gains_n = [10, 1 , 0.5, -0.01];    %20dB/decase

Freq_n  = optOptions.DesiredGndTrkLoopBW.*[(10^-1), (10^0), (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase


GndTrkLoopFreqShaping            =   TuningGoal.LoopShape('gndTrk_meas_rad',frd(Gains_n,Freq_n)); % loop transfer measured at theta_Deg (G*K)theta_Deg
GndTrkLoopFreqShaping.Focus      =   [10^-2, 5*10^0];
GndTrkLoopFreqShaping.Name       =   'GndTrk Outer Loop';
GndTrkLoopFreqShaping.Openings   =   {};        %{'Ail'} ;

% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 20% mismatch with reference model 1/(thau*s+1)
% GndTrkLoopTrackReq               = TuningGoal.StepTracking({'gndTrk_cmd_rad'},{'gndTrk_meas_rad'},tf(1,[25 1]));   %1/DesiredGndTrkLoop_Bandwidth is the time constant of the reference model
GndTrkLoopTrackReq               =  TuningGoal.StepTracking({'gndTrk_cmd_rad'},{'gndTrk_meas_rad'},1.0/optOptions.DesiredGndTrkLoopBW);   %1/DesiredGndTrkLoop_Bandwidth is the time constant of the reference model
GndTrkLoopTrackReq.Openings      =  {};           %{'Ail'} ;
GndTrkLoopTrackReq.RelGap        =  0.2;


% Margins Requirement
% Frequency-Loop shaping requirements
% Gain and phase margins at plant inputs and outputs
GndTrkLoopMarginReq1             = TuningGoal.Margins('phi_cmd_deg',optOptions.DesiredGndTrkLoopGM,optOptions.DesiredGndTrkLoopPM);
GndTrkLoopMarginReq1.Focus       = [10^-3, 4*10^1];
GndTrkLoopMarginReq1.Openings    = {};            %{'diffThrCmd_Norm'}  ; 

GndTrkLoopMarginReq2             = TuningGoal.Margins('gndTrk_meas_rad',optOptions.DesiredGndTrkLoopGM,optOptions.DesiredGndTrkLoopPM);
GndTrkLoopMarginReq2.Name        = 'GndTrk Outer Loop - Margin measured at the loop output (gndTrk_meas_rad)';
GndTrkLoopMarginReq2.Focus       = [10^-3, 4*10^1];
GndTrkLoopMarginReq2.Openings    = {};            %{'diffThrCmd_Norm'}  ; 


GndTrkLoop_TuningGoals_list=[GndTrkLoopFreqShaping, GndTrkLoopTrackReq, GndTrkLoopMarginReq1, GndTrkLoopMarginReq2];


Objectives_GndTrkLoop = [];

if( contains(GndTrkLoopTuningGoals,'Freq_Shaping') )

    Objectives_GndTrkLoop  =   [ Objectives_GndTrkLoop, GndTrkLoopFreqShaping ] ;
end

if( contains(GndTrkLoopTuningGoals,'Step_Tracking') )

    Objectives_GndTrkLoop  =   [ Objectives_GndTrkLoop, GndTrkLoopTrackReq ] ;
end

if( contains(GndTrkLoopTuningGoals,'Margins') )

    Objectives_GndTrkLoop  =   [ Objectives_GndTrkLoop, GndTrkLoopMarginReq1 ] ;

end

GndTrkLoop=struct;
GndTrkLoop.Name='GndTrkLoop';
GndTrkLoop.TuningGoals_list=GndTrkLoop_TuningGoals_list;
GndTrkLoop.Objectives=Objectives_GndTrkLoop;