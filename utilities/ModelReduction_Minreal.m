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


%% Perform pole / zero simplification on LTI system
load('sys_q_OL');
System = prescale(sys_q_OL); % Define System to reduce
Tol = 1e-08; % Set tolerance
 
% Simplify system poles/zeros
ReducedSystem = minreal(System,Tol);
 
% Create comparison plot
figure(1001);bode(System,ReducedSystem);

figure(1002);pzplot(System,'r',ReducedSystem,'b');legend('sys','reduced_sys')
damp(System);damp(ReducedSystem)