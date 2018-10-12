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
%% To get the transfer functions from gust to the displacement, velocity and acceleration outputs

load Aquila_1601.mat
aircraft_1601 = aircraft;

% Select trim case
ialt = 13;
ieas = 4;
igamma = 5;
iphi = 1;
idelta_spoiler = 1;
index = {ialt, ieas, igamma, iphi, idelta_spoiler};

plotter = TransferFunctionPlotter();
plotter.f_list = logspace(-2, 1, 1000);
plotter.show_figure = true;
plotter.save_to_file = false;

plotter.add_source(aircraft_1601.version,  aircraft_1601,  [0., 0., 1.], '-');

%% Plot particular loops of interest for a single model
plotter.path_output_base = pwd;

iinp = aircraft_1601.index.input.V_G;
iout = aircraft_1601.index.output_accel.z_ddot_imu;
otype = 'accel';
plotter.plot_input_to_output_loop(aircraft_1601.version, index, iinp, iout, otype);

iinp = aircraft_1601.index.input.V_G_dot;
iout = aircraft_1601.index.output_accel.z_ddot_imu;
otype = 'accel';
plotter.plot_input_to_output_loop(aircraft_1601.version, index, iinp, iout, otype);


%IMU Center node index (#1)
IMU_trans_idx=1:3;
IMU_rot_idx=4:6;
 
%Left Inner node index (#2)
LI_trans_idx=7:9;
LI_rot_idx=10:12;
 
%Left Outer node index (#3)
LO_trans_idx=13:15;
LO_rot_idx=16:18;
 
%Left Wingtip (#4)
Lwingtip_trans_idx=19:21;
Lwingtip_rot_idx=22:24;
 
 %Right Inner node index (#7)
RI_trans_idx=25:27;
RI_rot_idx=28:30;
 
%Right Outer node index (#6)
RO_trans_idx=31:33;
RO_rot_idx=34:36;
 
%Right Wingtip (#5)
Rwingtip_trans_idx=37:39;
Rwingtip_rot_idx=40:42;


%% from gust to the vertical acceleration or angular acceleration at the left wingtip ???
iinp = aircraft_1601.index;
iout = Lwingtip_trans_idx(3);
otype = 'accel';
plotter.plot_input_to_output_loop(aircraft_1601.version, index, iinp, iout, otype);



