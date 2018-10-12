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


% Configure the objective array for the Flexible Mode Control Loop


FlexLoopTuningGoals = cellarray2str(optOptions.FlexLoopTuningGoals) ;



% Margins Requirement
% Gain and phase margins at plant inputs and outputs
FlexLoopMarginReq1             = TuningGoal.Margins('elvCmd_flex_deg',optOptions.DesiredFlexLoopGM,optOptions.DesiredFlexLoopPM);
FlexLoopMarginReq1.Name        = 'Flexible Mode (Inner) Control Loop - Margin measured at the loop input (elvCmd_flex_deg)';
FlexLoopMarginReq1.Openings    = {};         %{'elvCmdDeg_PitchLoop','ailCmd_Deg'} ;

FlexLoopMarginReq1.Focus       = [(10^-1), 6*10^1];

FlexLoopMarginReq2             = TuningGoal.Margins('a_z_offcenter_meas_mpss',optOptions.DesiredFlexLoopGM,optOptions.DesiredFlexLoopPM);
FlexLoopMarginReq2.Name        = 'Flexible Mode (Inner) Control Loop - Margin measured at the loop output (a_z_offcenter_meas_mpss)';
FlexLoopMarginReq2.Openings    = {};         %{'elvCmdDeg_PitchLoop','ailCmd_Deg'} ;
FlexLoopMarginReq2.Focus       = [(10^-1), 6*10^1];

FlexLoop_TuningGoals_list=[FlexLoopMarginReq1, FlexLoopMarginReq2];


Objectives_FlexLoop = [];


if( contains(FlexLoopTuningGoals,'Margins') )

    Objectives_FlexLoop  =   [ Objectives_FlexLoop, FlexLoopMarginReq1, FlexLoopMarginReq2 ] ;

end

FlexLoop=struct;
FlexLoop.Name='FlexLoop';
FlexLoop.TuningGoals_list=FlexLoop_TuningGoals_list;
FlexLoop.Objectives=Objectives_FlexLoop;
