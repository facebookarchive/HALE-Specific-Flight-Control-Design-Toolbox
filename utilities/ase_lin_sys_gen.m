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


function [sys]=ase_lin_sys_gen(aircraft_model,trim_idx,node_info,prop_params,InputNames,OutputNames)

% This function generates linear ASE model with custom outputs
% Inputs :
%         aircraft_model:structure that stores aircraft object
%         trim_idx: array corresponding to trim condition.
%           1st element: alt index
%           2nd element: eas index
%           3rd element: gamma index
%           4th element: bank index
%           5th element: spoiler index
%         
%         node_info(1): output_at_node: flag to specify mean axis or node
%                       0: output at mean axis , 1:output at the node specified by node_idx
%         
%         node_info(2): node_idx: 1: IMU node
%                                 2: Left inner motor node
%                                 3: Left outer motor node
%                                 4: Left wingtip
%                                 5: Right wingtip
%                                 6: Right outer motor node
%                                 7: Right inner motor node
%                   
%
%                                  torque_const : relationship between throttle and torque . torque=torque_const*throttle
%         propulsion_parameters(1) is torque_const=45.2
%         propulsion_parameters(2) is throttle_mixing: mixing to be used between inner and outer motors
%                                  throttle_mixing= 0.36
%
%         
%
% sys.Outputs: sys object that stores linear A,B,C,D matrices
%           current outputs and ordering for output measurement is :
%          1: North, 
%          2: East , 
%          3: Altitude,
%          4: TAS,
%          5:roll,
%          6:pitch,
%          7:yaw,
%          8:P,
%          9:Q,
%          10:R,
%          11-13: V_NED (vector),
%          14:16: Imuaccel_b (vector),
%          17: Static Pressure,
%          18: Dynamic Pressure
%          19: OAT
%          20: EAS
%          21: AoA
%          22; AoS
%          23: U_Body = y_rel(1)*cos(aoa)*cos(aos) % these are TAS components in body 
%          24: V_Body = y_rel(1)*sin(aos)
%          25: W_Body = y_rel(1)*sin(aoa)*cos(aos)
%          26: Gamma ->  theta - Alpha

% system.inputs - the input names are given in "InputNames" vector
%     1: Collective Elevon
%     2: Differential Elevon
%     3: Collective Throttle 
%     4: Differential Throttle
%     5: Collective Spoiler  
%%% Example of use :
%           aircraft_model=load('Aquila_model');
%           trim_idx=[1 3 5 1 1];
%           node_info(1)=1; % output_at_node = 1 --> use node outputs (not mean axis)
%           node_info(2)=1; % node_idx=1  --> outputs at IMU node
%           prop_params(1)=45.2;
%           prop_params(2)=0.36;
%           [sys]=ase_lin_sys_gen(aircraft_model,trim_idx,node_info,propulsion_parameters,AquilaInputNames, AquilaOutputNames)

% Node displacements are provided in body coordinate wrt to mean axis
% Node velocity and accelerations are wrt inertial expressed expressed in body.
% currently assumed static pressure and OAT does not change with
% perturbations in state and control.

% Developer: Iman Alizade, Hamid Bolandhemmat
%indices of the output vector (row of matrix C) - is given in "OutputNames" vector

%          1: North, 
%          2: East , 
%          3: Altitude,
%          4: TAS,
%          5:roll,
%          6:pitch,
%          7:yaw,
%          8:P,
%          9:Q,
%          10:R,
%          ...
% 

output_at_node = node_info(1) ;
node_idx       = node_info(2) ;

torque_const    = prop_params(1);
throttle_mixing = prop_params(2);

output_roll_idx  = 5;
output_pitch_idx = 6;

output_p_idx     = 8;
output_q_idx     = 9;

%indices of the the (rigid body) state vector
p_N_idx=1;
p_E_idx=2;
h_idx=3;
roll_idx=4;   
pitch_idx=5;
yaw_idx=6;
VT_idx=7;
AoS_idx=8;
AoA_idx=9;
p_idx=10;
q_idx=11;
r_idx=12;

%indices of the the (flexible) state vector
EL_1ASB_idx = 15 ;      %1st Anti-symmetric Bending (EL_9)
EL_1SB_idx  = 16 ;      %1st Symmteric Bending (EL_10)
EL_2SB_idx  = 17 ;      %2nd Symmetric Bending (EL_11)
EL_1ST_idx  = 18 ;      %2nd Symmetric Bending (EL_12)


ELdot_1ASB_idx = 29 ;   %1st Anti-symmetric Bending (EL_dot_9)
ELdot_1SB_idx  = 30 ;   %1st Symmteric Bending (EL_dot_10)
ELdot_2SB_idx  = 31 ;      %2nd Symmetric Bending (EL_dot_11)
ELdot_1ST_idx  = 32 ;      %2nd Symmetric Bending (EL_dot_12)

ialt=trim_idx{1};
ieas=trim_idx{2};
igamma=trim_idx{3};
iphi=trim_idx{4};
ispoiler=trim_idx{5};


xTrim=aircraft_model.linear.trim(ialt, ieas, igamma, iphi,ispoiler).x;
uTrim=aircraft_model.linear.trim(ialt, ieas, igamma, iphi,ispoiler).u;
n_state=length(aircraft_model.definition.state.label);
n_control=length(aircraft_model.definition.input.label);
[~, ~, ~, y_rel, ~, ~] = aircraft_model.nonlinear.compute_output(xTrim, uTrim);
aircraft_model.linearize(ialt, ieas, igamma, iphi,ispoiler);
lm = aircraft_model.linear.model(ialt, ieas, igamma, iphi,ispoiler);
C_ned_to_body=angle2dcm(xTrim(6),xTrim(5),xTrim(4)); % mean axis attitude
[~, ~, ~, rho_trim] = atmosisa(xTrim(3));

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


if (output_at_node==1) && (node_idx==1)          %IMU node
    trans_idx=IMU_trans_idx;
    rot_idx=IMU_rot_idx;
elseif (output_at_node==1) && (node_idx==2)      %Left Inner node
    trans_idx=LI_trans_idx;
    rot_idx=LI_rot_idx;
elseif (output_at_node==1) && (node_idx==3)      %Left Outer node
    trans_idx=LO_trans_idx;
    rot_idx=LO_rot_idx;
elseif (output_at_node==1) && (node_idx==4)      %Left Wingtip
    trans_idx=Lwingtip_trans_idx;
    rot_idx=Lwingtip_rot_idx;
elseif (output_at_node==1) && (node_idx==5)     %Right Wingtip
    trans_idx=Rwingtip_trans_idx;
    rot_idx=Rwingtip_rot_idx;
elseif (output_at_node==1) && (node_idx==6)     %Right Outer node
    trans_idx=RO_trans_idx;
    rot_idx=RO_rot_idx;
elseif (output_at_node==1) && (node_idx==7)     %Right Inner node
    trans_idx=RI_trans_idx;
    rot_idx=RI_rot_idx;
end
    
tas=y_rel(1);  %this has the effect of wind speed included
aos=y_rel(2);
aoa=y_rel(3);


C_delta_EAS=lm.C_rel(1,:).*sqrt(rho_trim./1.225);
C_delta_u=lm.C_rel(1,:).*cos(aoa).*cos(aos)-lm.C_rel(3,:).*tas.*sin(aoa).*cos(aos)-lm.C_rel(2,:).*tas.*cos(aoa).*sin(aos);
C_delta_v=lm.C_rel(1,:).*sin(aos)+tas.*lm.C_rel(2,:).*cos(aos);
C_delta_w=lm.C_rel(1,:).*sin(aoa).*cos(aos)+lm.C_rel(3,:).*tas.*cos(aoa).*cos(aos)-lm.C_rel(2,:).*tas.*sin(aoa).*sin(aos);
C_delta_aoa=lm.C_rel(3,:);
C_delta_aos=lm.C_rel(2,:);
C_delta_gamma=[zeros(1,pitch_idx-1) 1 zeros(1,n_state-pitch_idx)]-C_delta_aoa; % defined at mean axis since aoa is scalar and does not depend on the node location in ASE model
D_delta_EAS=lm.D_rel(1,:).*sqrt(rho_trim./1.225);
D_delta_u=lm.D_rel(1,:).*cos(aoa).*cos(aos)-lm.D_rel(3,:).*tas.*sin(aoa).*cos(aos)-lm.D_rel(2,:).*tas.*cos(aoa).*sin(aos);
D_delta_v=lm.D_rel(1,:).*sin(aos)+tas.*lm.D_rel(2,:).*cos(aos);
D_delta_w=lm.D_rel(1,:).*sin(aoa).*cos(aos)+lm.D_rel(3,:).*tas.*cos(aoa).*cos(aos)-lm.D_rel(2,:).*tas.*sin(aoa).*sin(aos);
D_delta_aoa=lm.D_rel(3,:);
D_delta_aos=lm.D_rel(2,:);
D_delta_gamma=zeros(1,n_control)-D_delta_aoa; % 


switch output_at_node
    
    case 0 % mean axis

        C_lin=[1 zeros(1,n_state-p_N_idx);...
               zeros(1,p_E_idx-1) 1  zeros(1,n_state-p_E_idx);...
               zeros(1,h_idx-1) 1 zeros(1,n_state-h_idx);...
               lm.C_rel(1,:);...
               zeros(1,roll_idx-1) 1 zeros(1,n_state-roll_idx);...
               zeros(1,pitch_idx-1) 1 zeros(1,n_state-pitch_idx);...
               zeros(1,yaw_idx-1) 1 zeros(1,n_state-yaw_idx);...
               zeros(1,p_idx-1) 1 zeros(1,n_state-p_idx);...
               zeros(1,q_idx-1) 1 zeros(1,n_state-q_idx);...
               zeros(1,r_idx-1) 1 zeros(1,n_state-r_idx);...
               lm.A(p_N_idx,:);...
               lm.A(p_E_idx,:);...
               -lm.A(h_idx,:);...
               lm.C_accel(IMU_trans_idx,:);...  %% assummed mean axis accel is the same as IMU
               zeros(1,n_state);...
               rho_trim.*tas.*lm.C_rel(1,:);...
               zeros(1,n_state);...
               C_delta_EAS;...
               C_delta_aoa;...
               C_delta_aos;...
               C_delta_u;...
               C_delta_v;...
               C_delta_w;...
               C_delta_gamma];

               D_lin = [zeros(3,n_control);...
                        lm.D_rel(1,:);...
                        zeros(3,n_control);...
                        zeros(3,n_control);...
                        zeros(3,n_control);...
                        lm.D_accel([1,2,3],:);...
                        zeros(1,n_control);...
                        rho_trim.*tas.*lm.D_rel(1,:);...
                        zeros(1,n_control);...
                        D_delta_EAS;...
                        D_delta_aoa;...
                        D_delta_aos;...
                        D_delta_u;...
                        D_delta_v;...
                        D_delta_w;...
                        D_delta_gamma];            
                 
                

    case 1 % node
    
        C_lin = [C_ned_to_body'*lm.C_disp(trans_idx,:);...      % North, East and altitude outputs
                lm.C_rel(1,:);...                               % TAS 
                lm.C_disp(rot_idx,:);...                        % roll/pitch/yaw
                lm.C_vel(rot_idx,:);...                         % p/q/r of the node
                C_ned_to_body'*lm.C_vel(trans_idx,:);...        % velocity w.r.t the NED frame
                lm.C_accel(trans_idx,:);...                     % absolute acceleration also expressed in the body mean axes frame
                zeros(1,n_state);...
                rho_trim.*y_rel(1).*lm.C_rel(1,:);...
                zeros(1,n_state);...
                C_delta_EAS;...
                C_delta_aoa;...
                C_delta_aos;...
                C_delta_u;...
                C_delta_v;...
                C_delta_w;...
                C_delta_gamma]; % note that C_delta_gamma is defined at mean axis since there is no alpha for IMU separateley
    
                C_lin(h_idx,:)= -C_lin(h_idx,:); % in above the 3rd row would be positive in down direction
                             % however when multiplying by delta_h we should like positive to be upward
                
                %add effect of the mean axis states             
                C_lin(1,p_N_idx)= C_lin(1,p_N_idx) + 1; % North
                C_lin(2,p_E_idx)= C_lin(2,p_E_idx) + 1; % East
                C_lin(3,h_idx) = C_lin(3,h_idx) + 1; % alt
                C_lin(5,roll_idx) = C_lin(5,roll_idx) + 1; % roll
                C_lin(6,pitch_idx) = C_lin(6,pitch_idx) + 1; % pitch
                C_lin(7,yaw_idx) = C_lin(7,yaw_idx) + 1; % yaw

                D_lin = [C_ned_to_body'*lm.D_disp(trans_idx,:);...
                        lm.D_rel(1,:);...
                        lm.D_disp(rot_idx,:);...
                        lm.D_vel(rot_idx,:);...
                        C_ned_to_body'*lm.D_vel(trans_idx,:);...
                        lm.D_accel(trans_idx,:);...
                        zeros(1,n_control);...
                        rho_trim.*y_rel(1).*lm.D_rel(1,:);...
                        zeros(1,n_control);...
                        D_delta_EAS;...
                        D_delta_aoa;...
                        D_delta_aos;...
                        D_delta_u;...
                        D_delta_v;...
                        D_delta_w;...
                        D_delta_gamma]; 
                        D_lin(h_idx,:)= -D_lin(h_idx,:); % same logic for C_lin(3,:) 
     
     
end

% HB - 1. row numbers of the B_mixing matrix correspond to the particular inputs in the Aquila input vector described in table 3.2 of the model manual 
%      2. column numbers of the B_mixing matrix correspond to the
%      particular inputs in the final/new input vector (which is used in the Simulink model) --> B_mixing(row index of the original input vector,row index of the new input vector)
%      3. Note: for adding any new input to the Simulink model input vector, a new row must be added here to the B_mixing matrix
if n_control==41
    B_mixing = zeros(n_control,4);
else
    B_mixing = zeros(n_control,6); % spoiler/vertical gust input
    B_mixing(42:43,5)=[1;1];
end

B_mixing(1,1) = 1;
B_mixing(2,2) = 1;
% HB - New ordering and mixing based on the current motors ordering and direction sign - note that the first two torques correspond to the 
%      left motors rotating CCW (negative) - also positive rudder gives positive yaw rate
B_mixing(6:9,3:4) = torque_const.*[-1,-1  ;   -1,-throttle_mixing   ;   1,-throttle_mixing    ;    1,-1];  

%Including the vertical gust input 
B_mixing(3,6) = 1;  

A_lin=lm.A;    % HB - States here are the rigid body states + elastic states
B_lin=lm.B;
sys=ss(A_lin,B_lin,C_lin,D_lin)*B_mixing;

sys.InputName  = InputNames ;
sys.OutputName = OutputNames ; 

for i=1:length(aircraft_model.definition.state.label)
    state_labels(i) = aircraft_model.definition.state.label{i};
    state_lags(i)   = strncmp('lag',state_labels(i),3);
end

sys.StateName  = state_labels ;

%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% This segment constructs a scaled state space realization from the (lag) residualized system for the Modal analysis
% New states will have physical interpretations with engineering units

%residualize the MIMO model - Quasi-steady aerodynamic model
% sys_modred_residual = modred(sys,state_lags, 'matchdc');
sys_modred_residual = modred(sys,state_lags, 'matchdc');   %perform the modred command on the specific input-output pair

%truncate the inertial position states
%Also truncate the 3 first inertial position states:
elim=(1:3);
sys_modred_residual = modred(sys_modred_residual,elim,'truncate');

%indices of the the output vector in the residualized system
output_roll_idx  = 5;    %for the residualized model
output_pitch_idx = 6;

output_p_idx     = 8;
output_q_idx     = 9;

%indices of the the (rigid body) state vector in the residualized system
% p_N_idx=1;  %residualized
% p_E_idx=2;  %residualized
% h_idx=3;    %residualized

roll_idx=1;   
pitch_idx=2;
yaw_idx=3;
VT_idx=4;
AoS_idx=5;
AoA_idx=6;
p_idx=7;
q_idx=8;
r_idx=9;
              % lag states are residualized
%indices of the the (flexible) state vector
EL_1ASB_idx = 12 ;      %1st Anti-symmetric Bending (EL_9)
EL_1SB_idx  = 13 ;      %1st Symmteric Bending (EL_10)
EL_2SB_idx  = 14 ;      %2nd Symmetric Bending (EL_11)
EL_1ST_idx  = 15 ;      %2nd Symmetric Bending (EL_12)


ELdot_1ASB_idx = 26 ;   %1st Anti-symmetric Bending (EL_dot_9)
ELdot_1SB_idx  = 27 ;   %1st Symmteric Bending (EL_dot_10)
ELdot_2SB_idx  = 28 ;      %2nd Symmetric Bending (EL_dot_11)
ELdot_1ST_idx  = 29 ;      %2nd Symmetric Bending (EL_dot_12)

% State vector for Modal analysis:
% Define indices of the new state vector
modal_VT_idx        = 1 ;        % V_T (mps)                                               ----> original state #7
modal_AoA_idx       = 2 ;        % alpha ()                                                ----> original state #9
modal_RBpitch_idx   = 3 ;        % theta (rad) - mean axis (rigid body)                    ----> original state #5
modal_RBq_idx       = 4 ;        % theta_dot (rad/s) - mean axis (rigid body)              ----> original state #11
modal_EBpitch_idx   = 5 ;        % theta_E (rad)- @ node (elastic deformation)             ----> from the output matrix
modal_EBq_idx       = 6 ;        % theta_E_dot (rad/sec) - @ node (elastic deformation)    ----> from the output matrix

n_Modal = 6 ;

% Determine the desired rigid body/flexible states (in the longitudinal
%channel only for now ) to be picked from the original state matrix

% Choose a subset of the original state vector indices in the desired order for the Modal analysis 
reduced_states_indices = [VT_idx, AoA_idx, pitch_idx, q_idx, EL_1SB_idx, ELdot_1SB_idx] ;

% A subset of the original state matrix containing the Rigid Body (RB) and
% Elastic Body (EB - unit less) states: 
A_modal = sys_modred_residual.A(reduced_states_indices,reduced_states_indices) ;

% Construct the transformation matrix to replace the unitless elastic states with new elastic states having engineering units 
Trans_Mtx_unit = eye(n_Modal,n_Modal) ;

% X_new = Trans_Mtx * X_old
Trans_Mtx_unit(modal_EBpitch_idx, :)   =   sys_modred_residual.C(output_pitch_idx,reduced_states_indices) ;
Trans_Mtx_unit(modal_EBq_idx, :)       =   sys_modred_residual.C(output_q_idx,reduced_states_indices) ;

Trans_Mtx_unit_inv = inv(Trans_Mtx_unit) ;

% Construct the new state matrix for the new states with units:
A_modal_unit = Trans_Mtx_unit * A_modal * Trans_Mtx_unit_inv ;

% Construct a new transformation matrix to balance significance of the units
m2ft    = 3.28 ;
rad2deg = 180/pi ;

Trans_Mtx_scale       = diag([m2ft; rad2deg ; rad2deg ; rad2deg ; rad2deg ; rad2deg ]);
Trans_Mtx_scale_inv   = inv(Trans_Mtx_scale) ;

A_modal_scale   = Trans_Mtx_scale * A_modal_unit * Trans_Mtx_scale_inv ;


%Get the state matrix eigen structure - V is matrix of the eigen vectors and D is the diagonal eigen value matrix 
[V,D] = eig(A_modal_scale) ;     %A*V = V*D
 
%convert the eigen vector Matrix into the polar coordinates:
n_V = size(V,1);
V_polar_mag   = zeros(n_V,n_V);
V_polar_ang   = zeros(n_V,n_V);

for i=1:n_V
    for j=1:n_V
        [V_polar_ang(i,j),V_polar_mag(i,j)] = cart2pol(real(V(i,j)),imag(V(i,j))) ;
        V_polar_ang(i,j)  = V_polar_ang(i,j)*rad2deg ;
    
    end
end
