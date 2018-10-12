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



% Simulink model initialization                                           

%for the actual model - with the IMU sensor at the centerline
LIN_AC_S_A = ones(112,112) ;

LIN_AC_S_B = ones(112,6) ;

LIN_AC_S_C = ones(26,112) ;

LIN_AC_S_D = ones(26,6) ;


%Model for the flexible modes control - with the sensors installed off the centerline
% NOTE AM: Why were these sizes set to 1?
LIN_FLEX_AC_S_A = ones(112,112) ;
  
LIN_FLEX_AC_S_B = ones(112,6) ;

LIN_FLEX_AC_S_C = ones(26,112) ;

LIN_FLEX_AC_S_D = ones(26,6) ;
