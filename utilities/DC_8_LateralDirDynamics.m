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


s=tf('s');

T_Phi_Ail_OL   = 1.62*(s^2 + 0.362*s + 1.359)/( (s+0.0065)*(s+1.329)*(s^2+0.254*s+1.433) ) ;

T_p_Ail_OL     = 1.62*s*(s^2 + 0.362*s + 1.359)/( (s+0.0065)*(s+1.329)*(s^2+0.254*s+1.433) ) ;


T_beta_Ail_OL  = 0.0188*((s+0.197)*(s-7.896))/( (s+0.0065)*(s+1.329)*(s^2+0.254*s+1.433) ) ;


T_p_Rdr_OL     = 0.392*s*((s+1.85)*(s-2.566))/( (s+0.0065)*(s+1.329)*(s^2+0.254*s+1.433) ) ;


T_r_Rdr_OL     = 0.864*((s+1.335)*(s^2-0.03*s+0.109))/( (s+0.0065)*(s+1.329)*(s^2+0.254*s+1.433) ) ;

figure(3111);bode(T_Phi_Ail_OL,'r',T_p_Ail_OL,'b',T_p_Rdr_OL,'g');legend('OL Ail to roll attitude','OL Ail to Roll rate','OL Rdr to Roll Rate (X-couling)');grid;

figure(3112);bode(T_Phi_Ail_OL,'r',T_p_Ail_OL,'b',T_p_Rdr_OL,'g',T_beta_Ail_OL,'k');legend('OL Ail to roll attitude','OL Ail to Roll rate','OL Rdr to Roll Rate (X-couling)','OL Ail to side slip angle');grid;

figure(3113);bode(T_r_Rdr_OL,'r');legend('OL Rdr to yaw rate');grid;

T_nonminphase = (s-7)/(s+21) ;
T_minphase    = (s+7)/(s+21) ;

figure(3114);bode(T_nonminphase,'r',T_minphase,'b')