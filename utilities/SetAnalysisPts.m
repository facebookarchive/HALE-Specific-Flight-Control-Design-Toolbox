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
%   Add the analysis points for the Pitch Loop
addPoint(ST0,{'theta_ref','theta_Deg','theta_err','theta_meas','theta_intgd_deg'});

% Second, adding the signals for the inner loop analysis and inner loop bandwidth
% requirements, 
% Designate as analysis points the plant inputs and outputs (control and
% measurement signals) where the stability margins are measured. 

addPoint(ST0,{'elv_Cmd_ThetaLoop','elv_Cmd_qLoop','q_Dps','q_meas','elvCmd_Deg','elv','elvCmdDeg_PitchLoop','in_dist_elv'}); %'qFiltWashed'

addPoint(ST0,{'pres_alt_m','theta_rad','q_radps','DynPres_Pa'});


% Add the analysis points for the Climb rate Loop
addPoint(ST0,{'climb_rate_cmd_spdloop_mps','climb_rate_mps', 'climb_rate_spdloop_error_mps', 'theta_cmd_uncapped_deg','climb_rate_meas_mps'});

% Add the analysis points for the TAS Loop
addPoint(ST0,{'tas_cmd_mps','tas_mps', 'tas_error_mps', 'tas_adv_mps','tas_meas_mps' ,'climb_rate_cmd_spdloop_uncapped_mps'});

% Add the analysis points for the Altitude Loop
addPoint(ST0,{'alt_cmd_m','presAlt_meas_m','alt_error_m','climb_rate_cmd_altloop_uncapped_mps','climb_rate_cmd_altloop_mps'});

% Add the analysis points for the Power control Loop
addPoint(ST0,{'climb_rate_altloop_error_mps','CollThrCmd_uncapped_norm','CollThrCmdNorm_PwrLoop','in_dist_thr','thrCmd_Norm'});

% Add the analysis points for the lateral control Loop
addPoint(ST0,{'phi_rad','phi_deg','p_radps','p_dps','phi_meas_deg','rollRate_meas_dps'});

addPoint(ST0,{'phi_ref','phi_err_filt','phi_intgd_deg','aos_deg'});

addPoint(ST0,{'ailCmd_Deg','in_dist_ail','ail_Cmd_PhiLoop','ail_Cmd_pLoop','ailCmd_rollLoop_uncapped_deg','Ail'});

% Add the analysis points for the Gnd Trk control Loop
addPoint(ST0,{'gndTrk_cmd_rad','gndTrk_meas_rad'});

addPoint(ST0,{'phi_cmd_uncapped_deg','phi_cmd_deg','phiErrAdv_cmd'});


% Add the analysis points for the directional control Loop
addPoint(ST0,{'psi_rad','psi_deg','r_radps','r_dps'});

addPoint(ST0,{'yaw_meas_deg','yawRate_meas_dps'});

addPoint(ST0,{'yawRate_ref_dps','yawRate_ref_uncapped_dps','yawRate_err','rdr_cmd_uncapped_norm','rdr_cmd_norm'});

addPoint(ST0,{'diffThrCmd_Norm'});


%Add the analysis points for the The Spoiler loop
addPoint(ST0,{'splCmd_Deg','nz_GLAblended_mpss','splCmd_GLA_deg'});

%Add the analysis point for the Flex control loop
addPoint(ST0,{'a_x_IMU_mpss','a_z_IMU_mpss','a_z_offcenter_mpss','a_z_offcenter_meas_mpss','elvCmd_flex_deg'});

%Add the analysis points for the Gust Load Alleviation (GLA)
addPoint(ST0,{'vertGust_mps'});

%%Display the analysis points:
AnalysisPts = getPoints(ST0) ;
%disp(AnalysisPts) ;