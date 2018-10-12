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
function sys_new = sysTransformTwist(sys)
    % Reorder the sys from ASWING so that the inputs and outputs match the 
    % format for the control toolbox
    
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    
    throttleGain = 10;
    
    numStates = numel(A(:,1));
    numInputs = numel(B(1,:));
    numOutputs = numel(C(:,1));
    
    % A matrix is fine, we don't touch the states
    
    % B - Reorder inputs
    B_new = B(:,[2 1 4 3]); % elev,ail,thr,rudder
    B_new = [B_new zeros(numStates,2)]; % ASWING model doesn't have spoiler or gust, set to zero
    B_new(:,1:2) = rad2deg(B_new(:,1:2)); % Angle conversions for elevator, aileron
    B_new(:,3) = B_new(:,3)*throttleGain; % Gain for normalized throttle input, should map to N away from trim
    B_new(:,4) = B_new(:,4)*15; % Convert normalized diff throttle to rudder in degrees
    
    % C - Calculate Twist
    C_new = [];
    C_new = [C_new; C(2,:)-C(4,:)]; % Right wing tip theta - root theta
    C_new = [C_new; C(3,:)-C(4,:)]; % Left wing tip theta - root theta
    C_new = [C_new; zeros(24,numStates)];
    
    % D - Reorder inputs and outputs
    D_new = [];
    D_new = [D_new; D(2,:)-D(4,:)]; % Right wing tip theta - root theta
    D_new = [D_new; D(3,:)-D(4,:)]; % Left wing tip theta - root theta
    D_new = [D_new; zeros(24,numInputs)];
    % Reorder inputs
    D_new = D_new(:,[2 1 4 3]); % elev,ail,thr,rudder
    D_new = [D_new zeros(26,2)]; % ASWING model doesn't have spoiler or gust, set to zero
    % Units and scaling
    D_new(:,1:2) = rad2deg(D_new(:,1:2)); % Angle conversions for elevator, aileron
    D_new(:,3) = D_new(:,3)*throttleGain; % Gain for normalized throttle input, should map to N away from trim
    D_new(:,4) = D_new(:,4)*15; % Convert normalized diff throttle to rudder in degrees
    
    % Combine
    sys_new = ss(A,B_new,C_new,D_new); 
%     sys_new = prescale(sys_new);
    
%     sys_new.InputName = {'Elevator','Aileron','Motor','Rudder','Spoiler','Gust'};
%     sys_new.OutputName = {'p_n','p_e','pres_alt','tas_mps','phi_rad','theta_rad','psi_rad','p_rad/s','q_rad/s','r_rad/s','v_n_mps','v_e_mps','v_d_mps','a_x','a_y','a_z','_','dyn_pres','OAT','eas_mps','aoa_rad','aos_rad','ux','uy','uz','gamma_rad'};
    
    sys_new.InputName = {'elvCmd_rad' 'ailCmd_rad' 'thrCmd_Norm' 'diffThrCmd_Norm' 'splCmd_rad','vertGust_mps'};       %Names of the Aquila model input commands in the Simulink model 
    sys_new.OutputName = {'twist_right_wing_deg','twist_left_wing_deg','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a','n/a'}; %Names of the Aquila model output signals in the Simulink model

end