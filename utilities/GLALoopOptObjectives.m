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

% Configure the objective array for the GLA Control Loop


GLALoopTuningGoals = cellarray2str(optOptions.GLALoopTuningGoals) ;



% Margins Requirement
% Gain and phase margins at plant inputs and outputs
GLALoopMarginReq1             = TuningGoal.Margins('splCmd_Deg',optOptions.DesiredGLALoopGM,optOptions.DesiredGLALoopPM);
GLALoopMarginReq1.Name        = 'GLA (Inner) Control Loop - Margin measured at the loop input (splCmd_deg)';
GLALoopMarginReq1.Openings    = {};        %{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm'} ;

GLALoopMarginReq1.Focus       = [(10^-1), 6*10^1];

GLALoopMarginReq2             = TuningGoal.Margins('nz_GLAblended_mpss',optOptions.DesiredGLALoopGM,optOptions.DesiredGLALoopPM);
GLALoopMarginReq2.Name        = 'GLA (Inner) Control Loop - Margin measured at the loop output (nz_GLAblended_mpss)';
GLALoopMarginReq2.Openings    = {};        %{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm'} ;
GLALoopMarginReq2.Focus       = [(10^-1), 6*10^1];

GLALoop_TuningGoals_list=[GLALoopMarginReq1, GLALoopMarginReq2];


Objectives_GLALoop = [];


if( contains(GLALoopTuningGoals,'Margins') )

    Objectives_GLALoop  =   [ Objectives_GLALoop, GLALoopMarginReq1, GLALoopMarginReq2 ] ;

end

GLALoop=struct;
GLALoop.Name='GLALoop';
GLALoop.TuningGoals_list=GLALoop_TuningGoals_list;
GLALoop.Objectives=Objectives_GLALoop;
