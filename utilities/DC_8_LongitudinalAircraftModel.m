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


% Define the aircraft model

kq_elv = -10.45 ;

%zeros
z0 =  0.0 ;
z1 = -0.9871 ;
z2 = -0.02179 ;

Zq_elv = [z0, z1, z2] ;

% Short period poles
p1 = -1.204+j*1.492 ;
p2 = -1.204-j*1.492 ;

% Phugoid poles
p3 = -0.007654+j*0.07812 ;
p4 = -0.007654-j*0.07812 ;

Pq_elv = [p1, p2, p3, p4] ;

% Gq_elv = zpk(Zq_elv,Pq_elv,kq_elv);
% P = bodeoptions ;
% P.Grid = 'on' ;
% P.PhaseVisible  = 'on' ;
% P.PhaseWrapping = 'off' ;
% figure(1);bode(Gq_elv,P);
% 
% 
% [LIN_AC_S_A,LIN_AC_S_B,LIN_AC_S_C,LIN_AC_S_D] = zp2ss(Zq_elv,Pq_elv,k);

% The 
LIN_AC_S_A = [

   -0.0153   -0.0785         0         0;
    0.0785         0         0         0;
    1.0000         0   -2.4080   -1.9172;
         0         0    1.9172         0];


LIN_AC_S_B =[

     1;
     0;
     0;
     0];


LIN_AC_S_C = [

  -10.4500         0   14.6207   19.9176];


LIN_AC_S_D =0;
