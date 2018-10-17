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
%   Developer: Abe Martin
function sys_new = sysTransform(sys,height)
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
    
    % C - Reorder outputs
    C_new = C([10 11 12 16 13 14 15 7 8 9 4 5 6 1 2 3],:); 
    C_new = [C_new; zeros(3,numStates)]; % Set unused air outputs to 0
    C_new = [C_new; C([16 18 17],:)];
    C_new = [C_new; zeros(4,numStates)]; % Set unused body frame velocities and gamma to zero
    C_new(5:10,:) = deg2rad(C_new(5:10,:)); % Angle conversion for phi theta psi Wx Wy Wz
    C_new(21:22,:) = deg2rad(C_new(21:22,:)); % Angle conversion for alpha beta
    C_new([1,8,10,11,13,14,16],:) = -(C_new([1,8,10,11,13,14,16],:)); % Reflect ASWING x,z axis
    % RX,Wx,Wz,UX,UZ,ax,az, - RZ-> pres_alt
    % EAS Conversion
    [rho_0,~,~,~,~,~] = atmos(0);
    [rho,~,~,~,~,~] = atmos(height);
    EAS_ratio = sqrt(rho/rho_0);
    C_new(20,:) = C_new(20,:)*EAS_ratio;  % TAS to EAS
    
    
    % D - Reorder inputs and outputs
    % Reorder outputs
    D_new = D([10 11 12 16 13 14 15 7 8 9 4 5 6 1 2 3],:);
    D_new = [D_new; zeros(3,4)]; % Set unused air outputs to 0
    D_new = [D_new; D([16 18 17],:)];
    D_new = [D_new; zeros(4,4)]; % Set unused body frame velocities and gamma to zero
    % Reorder inputs
    D_new = D_new(:,[2 1 4 3]); % elev,ail,thr,rudder
    D_new = [D_new zeros(26,2)]; % ASWING model doesn't have spoiler or gust, set to zero
    % Units and scaling
    D_new(:,1:2) = rad2deg(D_new(:,1:2)); % Angle conversions for elevator, aileron
    D_new(5:10,:) = deg2rad(D_new(5:10,:)); % Angle conversion for phi theta psi Wx Wy Wz
    D_new(21:22,:) = deg2rad(D_new(21:22,:)); % Angle conversion for alpha beta
    D_new([1,8,10,11,13,14,16],:) = -(D_new([1,8,10,11,13,14,16],:)); % Reflect ASWING x axis for velocity, acc, and position
    D_new(20,:) = D_new(20,:)*EAS_ratio;  % TAS to EAS
    D_new(:,3) = D_new(:,3)*throttleGain; % Gain for normalized throttle input, should map to N away from trim
    D_new(:,4) = D_new(:,4)*15; % Convert normalized diff throttle to rudder in degrees
    
    % Augment with new states / replace bad position with integrated
    % velocity
    A_new = A;
    A_new = [A_new; C_new([11:13,10],:)]; % Augment new p_N, p_E, press_alt, psi
    A_new = [A_new zeros(size(A_new,1),4)]; % Add zeros to account for new states
    B_new = [B_new; D_new([11:13,10],:)]; % Augment new p_N, p_E, press_alt, psi

    C_new = [C_new zeros(size(C_new,1),4)]; % Add zeros to account for new states
    C_new([1:3,7],:) = 0; % Clear old outputs
    C_new(1,end-3) = 1; % Add new p_N output
    C_new(2,end-2) = 1; % Add new p_E output
    C_new(3,end-1) = -1; % Add new pres_alt output
    C_new(7,end) = -1; % Add new psi output

    D_new([1:3,7],:) = 0; % Clear old passthroughs 
    
    % Combine
    sys_new = ss(A_new,B_new,C_new,D_new); 
%     sys_new = prescale(sys_new);
    
%     sys_new.InputName = {'Elevator','Aileron','Motor','Rudder','Spoiler','Gust'};
%     sys_new.OutputName = {'p_n','p_e','pres_alt','tas_mps','phi_rad','theta_rad','psi_rad','p_rad/s','q_rad/s','r_rad/s','v_n_mps','v_e_mps','v_d_mps','a_x','a_y','a_z','_','dyn_pres','OAT','eas_mps','aoa_rad','aos_rad','ux','uy','uz','gamma_rad'};
    
    sys_new.InputName = {'elvCmd_rad' 'ailCmd_rad' 'thrCmd_Norm' 'diffThrCmd_Norm' 'splCmd_rad','vertGust_mps'};       %Names of the Aquila model input commands in the Simulink model 
    sys_new.OutputName = {'p_n_m' 'p_e_m' 'pres_alt_m' 'tas_mps' 'phi_rad' 'theta_rad' 'psi_rad' 'p_radps' 'q_radps' 'r_radps' 'v_n_mps' 'v_e_mps' 'v_d_mps' 'a_x_mpss' 'a_y_mpss' 'a_z_mpss' 'StatPres_Pa' 'DynPres_Pa' 'OAT' 'eas_mps' 'aoa_rad' 'aos_rad' 'v_xb_mps' 'v_yb_mps' 'v_zb_mps' 'gama_rad'}; %Names of the Aquila model output signals in the Simulink model

end