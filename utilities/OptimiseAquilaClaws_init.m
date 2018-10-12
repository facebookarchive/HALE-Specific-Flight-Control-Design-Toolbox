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
%% Initialization function for the path, and the Simulink model parameters:
clear all;
close all;
clc

TRUE  = 1;
FALSE = 0 ; 
%Get the username of the user 
user_name = char(java.lang.System.getProperty('user.name'));

%set the path to the Aquila design folder -
Aquila_design_folder = 'Documents/aquila/control_folder' ;

main_folder=fullfile('/Users',user_name,Aquila_design_folder,'Aquila_Claws_Design_Cascaded');
addpath(genpath(main_folder));

AquilaDiscreteModelParameters;
