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
function run_aswing( aswing_opts )
%run_aswing runs aswing to output both a reduced order model and bode data
%at the desired conditions.  The outputs are saved to AF.bode, AF.e00, and
%AF.rom

%% ROM
% Read ROM input file
fid = fopen('ComROM_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'num_modes',num2str(aswing_opts.num_modes));

% Write new input file for aswing
fid = fopen('ComASE_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComASE_.txt');

%% ROM Twist
% Read ROM input file
fid = fopen('ComROMtwist_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name','AQ3mod.asw');
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'num_modes',num2str(aswing_opts.num_modes));

% Write new input file for aswing
fid = fopen('ComTwist_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTwist_.txt');

%% Bode
% Read Bode input file
fid = fopen('ComBode_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'num_modes',num2str(aswing_opts.num_modes));
f = strrep(f,'freq_min',num2str(aswing_opts.freq_min));
f = strrep(f,'freq_max',num2str(aswing_opts.freq_max));
f = strrep(f,'num_freq',num2str(aswing_opts.num_freq));

% Write new input file for aswing
fid = fopen('ComBode_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComBode_.txt');
end

