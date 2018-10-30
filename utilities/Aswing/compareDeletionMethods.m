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
load sys_aswing_delete.mat
load sys_manual_del.mat
load sys_modred_delete.mat

% Open bode file
fid2 = fopen('AQ3nonlin.bode');

in = {'Aileron','Elevator','Rudder','Motor'};
out = {'dUx','dUy','dUz','UX','UY','UZ','Wx','Wy','Wz','RX','RY','RZ','Phi','Theta','Psi','V','beta','alpha'};

Response2 = [];

% Parse Bode information from Aswing
for u = 1:numel(in)
    for y = 1:numel(out)
        % Read Aswing Nonlinear Frequency Response
        R = textscan(fid2,'%f %f %f',aswing_opts.num_freq,'CommentStyle','#','Delimiter','\t');
        R = [R{:}];
        Freq2 = R(:,1);
        Gain2 = R(:,2);
        Phase2 = R(:,3);
        % Convert response to complex from
        % rho e^(J theta) = rho * cos(theta) + j rho * sin(theta)
        Response2(y,u,:) = Gain2.*cos(deg2rad(Phase2))+1i*Gain2.*sin(deg2rad(Phase2));
    end 
end

% Convert frequency from Hz to rad/s
Freq2 = Freq2 * (2 * pi());

% Create frequency response object.
fr_data_nonlin = idfrd(Response2,Freq2,0);
fr_data_nonlin.InputName = in;
fr_data_nonlin.OutputName = out;
fr_data_nonlin.Name = 'ASWING Nonlinear';

in = 2;
out = 8;

figure
compare(fr_data_nonlin(out,in),sys_orig(out,in),sys_red(out,in),sys_del(out,in),'--');
legend('Aswing Nonlinear','Aswing Omit Mode','Modred','Manual Removal')
prettyPlots()

