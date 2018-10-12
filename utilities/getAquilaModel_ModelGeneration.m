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

%	PURPOSE:            1. Build and save open loop flight dynamics model for Aquila (Facebook High Altitude Long Endurance (HALE) very flexible airplane)
%                       
%
%	INPUTS:             1. Path to be added to the existing MATLAB paths (following the standard data folder address structure)
%                       2. aircraft build number (v1.3.2) and configuration
%                       3. version ('Aquila_')
%                       4. flight conditions to be analyzed.
% 
% 
% 	OUTPUTS:            1. The aircraft state space model at the given trim condition
%
%
%
%	PRE-REQUISITES:		ASE model should be available,  					
%
%	LIMITATIONS:		Currently many
%
%
%	DEVELOPER :		    H. Bolandhemmat, Abe Martin (Facebook Connectivity Lab)	
%
%
%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
clear all;
close all;
clc;

%% Custom Model

% IF USING A CUSTOM MODEL FROM ASWING, SET UseCustomModel TO 1
UseCustomModel = 1;

if(UseCustomModel==1)
    % Set options for ASWING model import 
    customModelName = 'ss_0_9rom';  % Choose model name
    
    % Choose whether to use system ID to estimate the sytem from aswing
    % bode data, or to use the ROM transfer method
    useSysID = false;
    runSysIDReport = false; % (0,1) Generate information and plots on sysID model fit

    % Aswing Options
    aswing_opts.asw_file_name = 'AQ3.asw';
    aswing_opts.point_file_name = 'SLF.pnt';
    aswing_opts.eas = 9;
    aswing_opts.altitude = 0;
    aswing_opts.num_modes = 60; % Number of modes to use in ROM model
    aswing_opts.freq_min = 0.00001;  % Frequency range for Aswing bode output (Hz)
    aswing_opts.freq_max = 5;
    aswing_opts.num_freq = 120; % Numer of frequencies to output (max 120 with unmodified aswing code)
    
    % Plot potentially bad IO channels from ROM
    debugROM = true;
    
    % Change to subfolder
    prev_folder = cd('./Aswing/');
    ASWING2MATLAB (customModelName,aswing_opts,debugROM,useSysID,runSysIDReport);
    % Move back to original folder
    cd(prev_folder);
else
% Load Normal model
%% path and addresses:
user_name                   = char(java.lang.System.getProperty('user.name'));
ACModelDataPath             = fullfile('/','Users',user_name,'Dropbox (Facebook)','Aquila','GNC','Modeling and Simulation','ASE','Aquila Matlab Simulator') ;
ACModelBuild                = 'v1.3.2';                      % It is usually defined within the model folder name (see the model folder name defined in the ACModelDataPath field)
ACModelVersion              = 'Aquila_';                     % Choose between the existing aircraft model versions


%Aircraft model inputs and outputs based on the settings in the ase_lin_sys_gen.m script and the Simulink linear model block
AquilaModelInputNames       = {'elvCmd_rad' 'ailCmd_rad' 'thrCmd_Norm' 'diffThrCmd_Norm' 'splCmd_rad','vertGust_mps'};       %Names of the Aquila model input commands in the Simulink model 
AquilaModelOutputNames      = {'p_n_m' 'p_e_m' 'pres_alt_m' 'tas_mps' 'phi_rad' 'theta_rad' 'psi_rad' 'p_radps' 'q_radps' 'r_radps' 'v_n_mps' 'v_e_mps' 'v_d_mps' 'a_x_mpss' 'a_y_mpss' 'a_z_mpss' 'StatPres_Pa' 'DynPres_Pa' 'OAT' 'eas_mps' 'aoa_rad' 'aos_rad' 'v_xb_mps' 'v_yb_mps' 'v_zb_mps' 'gama_rad'}; %Names of the Aquila model output signals in the Simulink model

% Settings to generate the state space (A,B,C, D) matrices with considering sensors either on the airplane mean axes or at 
% either of the nodes on the aircraft body (determined by node_idx)

%         output_at_node:       flag to specify mean axis or node
%                               0: output at mean axis , 1:output at the node specified by node_idx
%         
%         node_idx:             1: IMU node
%                               2: Left inner motor node
%                               3: Left outer motor node
%                               4: Left wingtip
%                               5: Right wingtip
%                               6: Right outer motor node
%                               7: Right inner motor node

output_at_node  = 1 ;

%Provide list of the aircraft analysis nodes here with the associated indeces:
Analysis_index_nodes   = {'IMU node';'Left inner motor node';'Left outer motor node';'Left wingtip';'Right wingtip';'Right outer motor node';'Right inner motor node'};
Nodes_index_rows       = {'1'       ; '2'                   ;'3'                    ;'4'           ;'5'            ;'6'                     ;'7'};          


% Aircraft model constants and parameters 
torque_const    = 45.2 ;
throttle_mixing = 0.36 ;


%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

sim_folder = fullfile(ACModelDataPath,'model','aircraft','ase','aquila_matlab_simulator') ;
addpath(genpath(sim_folder));

model_directory = fullfile(ACModelDataPath,'model','aircraft','ase','models', ACModelVersion);

Aquila = build_aircraft(ACModelVersion, ACModelBuild, model_directory);  

%record the desired flight condition with the proper name
disp('..........');
disp('  Saving the aircraft linearized models at different trim conditions in Aquila_OL_Model.mat ....');
current_time      =  datetime('now') ;
current_time_str  =  datestr(current_time) ;
current_time_str  =  strrep(current_time_str,' ','_');
current_time_str  =  strrep(current_time_str,'-','_');
current_time_str  =  strrep(current_time_str,':','_');

disp('..........');
disp('..........');
disp('  Including the built date (DD_MonthName_YYYYY) and time (HH_MM_SS) to the aircraft model name ....');
save(['Aquila_OL_Model_',current_time_str],'Aquila','AquilaModelInputNames','AquilaModelOutputNames','Analysis_index_nodes',...
                                           'output_at_node','torque_const','throttle_mixing','Nodes_index_rows','ACModelDataPath',...
                                           'ACModelBuild','ACModelVersion','model_directory','sim_folder');
                                       
end
