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
%   Configure the objective array for the Yaw Rate loop


YawRateLoopTuningGoals = cellarray2str(optOptions.YawRateLoopTuningGoals) ;

%Frequency-Loop shaping requirements
%     -20dB/decade within the bandwidth and then -40dB/decade beyond it;
%     Freq_n  = optOptions.DesiredYawLoopBW.*[(10^-1), (10^0), 2*(10^0), (10^1)] ;
%     Gains_n = [10, 1 , 0.5, -0.01];    %20dB/decase

Freq_n  = optOptions.DesiredYawRateLoopBW.*[(10^-1), (10^0), (10^1)] ;
Gains_n = [10, 1 , -0.1];    %20dB/decase


YawRateLoopFreqShaping            =   TuningGoal.LoopShape('yawRate_meas_dps',frd(Gains_n,Freq_n)); % loop transfer measured at yawRate_meas_dps (G*K)
YawRateLoopFreqShaping.Focus      =   [10^-2, (5*10^0)];
YawRateLoopFreqShaping.Name       =   'Yaw Rate Inner Loop';
YawRateLoopFreqShaping.Openings   =   {'yawRate_ref_uncapped_dps'};         % this loop should be open, otherwise the yaw rate loop is a derivative at low frequencies


% Step Tracking Requirement
% The tracking requirement should be in-line with the FreqShaping requirement:
% Less than 20% mismatch with reference model 1/(thau*s+1)
YawRateLoopTrackReq               = TuningGoal.StepTracking({'yawRate_ref_dps'},{'yawRate_meas_dps'},1.05/optOptions.DesiredYawRateLoopBW);   %1/DesiredYawRateLoop_Bandwidth is the time constant of the reference model
YawRateLoopTrackReq.Openings      = {'yawRate_ref_uncapped_dps'};  %{'phi_cmd_deg'} ;
YawRateLoopTrackReq.RelGap        = 0.2;

% Margins Requirement
%Frequency-Loop shaping requirements
% Gain and phase margins at plant inputs and outputs
YawRateLoopMarginReq1             = TuningGoal.Margins('diffThrCmd_Norm',optOptions.DesiredYawRateLoopGM,optOptions.DesiredYawRateLoopPM);
YawRateLoopMarginReq1.Name        = 'Yaw Rate Inner Loop - Margin measured at the loop input (diffThrCmd_Norm)';
YawRateLoopMarginReq1.Focus       = [(5*10^-2), 4*10^1];

YawRateLoopMarginReq2             = TuningGoal.Margins('yawRate_meas_dps',optOptions.DesiredYawRateLoopGM,optOptions.DesiredYawRateLoopPM);
YawRateLoopMarginReq2.Name        = 'Yaw Rate Inner Loop - Margin measured at the loop output (yawRate_meas_dps)';
YawRateLoopMarginReq2.Focus       = [(5*10^-2), 4*10^1];

YawRateLoop_TuningGoals_list=[YawRateLoopFreqShaping, YawRateLoopTrackReq, YawRateLoopMarginReq1, YawRateLoopMarginReq2];


Objectives_YawRateLoop = [];

if( contains(YawRateLoopTuningGoals,'Freq_Shaping') )

    Objectives_YawRateLoop  =   [ Objectives_YawRateLoop, YawRateLoopFreqShaping ] ;
end

if( contains(YawRateLoopTuningGoals,'Step_Tracking') )

    Objectives_YawRateLoop  =   [ Objectives_YawRateLoop, YawRateLoopTrackReq ] ;
end

if( contains(YawRateLoopTuningGoals,'Margins') )

    Objectives_YawRateLoop  =   [ Objectives_YawRateLoop, YawRateLoopMarginReq1, YawRateLoopMarginReq2 ] ;

end

YawRateLoop=struct;
YawRateLoop.Name='YawRateLoop';
YawRateLoop.TuningGoals_list=YawRateLoop_TuningGoals_list;
YawRateLoop.Objectives=Objectives_YawRateLoop;
