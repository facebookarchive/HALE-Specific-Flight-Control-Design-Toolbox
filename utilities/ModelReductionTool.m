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
%	SYNTAX
%            run by itself
%
%	PURPOSE: To reduce order of the model by different methods, and to compare the full order model 
%            and the reduced order model in frequency and time domains in order to analyze accuracy 
%            of the reduced order model
%
%	INPUTS:	Flight conditions, for which model reduction is required,
%           eventually this script is converted to a function which is revoked within the main design 
%           function 
%           
%
%	DEFAULT:
%
%	DEVELOPER :			H. Bolandhemmat (Facebook Connectivity Lab)
                      
%
%

clc;
close all;
%Provide the address to where aircraft model is stored - This 
user_name = char(java.lang.System.getProperty('user.name'));
path_folder=fullfile('/Users',user_name,'Aquila','GNC','Modeling and Simulation','ASE','Aquila Matlab Simulator');
addpath(genpath(path_folder));
%addpath(genpath(ACModelDataPath));
Aquila  = load('Aquila_.mat');

%Unit conversions
Deg2Rad = pi/180;
hsvd_offset = 0.0001 ; %offset to determine unstable poles

%Provide the flight condition of ineterst


alts   = [ 0 ] ;
eass   = [ 14.72  ] ;    %10.97,  14.72
gammas = [ 0 ] ;
phis   = [ 0 ] ;

ialt            = find(ismember(Aquila.aircraft.linear.index.alt, alts)) ;
ieas            = find(ismember(Aquila.aircraft.linear.index.eas, eass)) ;
igamma          = find(ismember(Aquila.aircraft.linear.index.gamma, gammas)) ;
iphi            = find(ismember(Aquila.aircraft.linear.index.phi, phis)) ;
idelta_spoiler  = 1;

state_0   = Aquila.aircraft.linear.trim(ialt, ieas, igamma, iphi).x;
u_0       = Aquila.aircraft.linear.trim(ialt, ieas, igamma, iphi).u;

[y_disp, y_vel, y_accel, y_rel, y_control, y_thrust]    = Aquila.aircraft.nonlinear.compute_output(state_0, u_0);

Flags_computed    = Aquila.aircraft.linear.trim(ialt, ieas, igamma, iphi).computed;
Flags_exist       = Aquila.aircraft.linear.trim(ialt, ieas, igamma, iphi).exist;

% Thrust and Torque 
Thrust_   = y_thrust' ;
Torque_   = u_0(6:9)' ;

delta_surface_    = (y_control/Deg2Rad)' ;                      %first element is right control surface and second is left

%Alpha and Beta trims
Alpha_trim_                  = y_rel(3)/Deg2Rad ;
Beta_trim_                   = y_rel(2)/Deg2Rad ;

%           1st element: alt index
%           2nd element: eas index
%           3rd element: gamma index
%           4th element: bank index
%           5th element: spoiler index

trim_idx = [ialt, ieas, igamma, iphi,idelta_spoiler ] ;
torque_const    = 45.2 ;
throttle_mixing = 0.36 ;
output_at_node  = 1 ;    %output_at_node: flag to specify mean axis or node
%                        0: output at mean axis , 1:output at the node specified by node_idx
[sys] = ase_lin_sys_gen(Aquila,trim_idx, output_at_node , 1 , 0 ,torque_const,throttle_mixing) ;  %torque_const=45.2
disp('full order model size:');
disp(size(sys.A));
% Get the linearzied input and feedforward matrices at the IMU location 
% [sys] = lin_outputmatrix_gen(Aquila,state_0,u_0,ialt, ieas, igamma, iphi,idelta_spoiler,1,1,0) ;

%% 1. Model reduction by simplification: minreal ->model reduction by cancelling near by pole/zero pairs, or the "non-minimal" state dynamics - all the uncontrollable
%               and unobservable modes have been removed.
%default Tolerance for zero-pole cancellation is TOL=SQRT(EPS), we reduce
%it to 1e-10
[sys_PbhStruct]=PbhTest(sys.A,sys.B,sys.C);
sys_minreal = minreal(sys,(1e-10));
disp('"pole-zero cancellation" reduced order model size:');
disp(size(sys_minreal.A));
[sys_minreal_PbhStruct]=PbhTest(sys_minreal.A,sys_minreal.B,sys_minreal.C);

%%2.a Model reduction by truncation and residulization: Balreal+Modred == Balred
% OPT = stabsepOptions('Offset', 1e-6 ,'RelTol', 1e-8);

opts = hsvdOptions('Offset',hsvd_offset); 
[sys_balreal,g] = balreal(sys,opts) ;                               % compute balanced realization
% g_noInf = g(~isinf(g)) ;                                          % remove the entries corrsponding to the unstable mdoes
% g_max           = max(g_noInf) ;
% g_normalized    = g./g_max ;

figure(200); pzmap(sys(9,1),'r',sys_balreal(9,1),'b');sgrid;legend('full order model','ful order model - Balanced');


elim = (g<1e-1) ;                                                 % identify states with smallest grammian g -> negligible states
% elim = (g_normalized<1e-4) ;                                        % identify states with relative small grammians (1% of the largest grammian g -> negligible states)
sys_modred_residual  =  modred(sys_balreal,elim,'matchdc');         % remove negligible states by using residulization, "matchdc" -> default
disp('"residulization" reduced order model size:');
disp(size(sys_modred_residual.A));
%[sys_modred_residual_PbhStruct]=PbhTest(sys_modred_residual.A,sys_modred_residual.B,sys_modred_residual.C);

sys_modred_trunct    =  modred(sys_balreal,elim,'truncate');        % remove negligible states by using truncation
disp('"Truncation" reduced order model size:');
disp(size(sys_modred_trunct.A));
%[sys_modred_trunct_PbhStruct]=PbhTest(sys_modred_trunct.A,sys_modred_trunct.B,sys_modred_trunct.C);

%% 3 Model reduction by similarity transformation

%%% reduce model size
fmax=3;  %reduced model Hz 
sys_modal=canon(sys,'modal', 1e6);  % Similarity transformation and canonical form
A_modal = sys_modal.a;
freqs   = sqrt(diag(A_modal).*diag(A_modal) + vertcat(diag(A_modal,1),  0.).*vertcat(diag(A_modal,1),  0.) + vertcat( 0., diag(A_modal,-1)).*vertcat( 0., diag(A_modal,-1)));
elim    = (freqs>fmax*2*pi);
sys_modred_modalred = modred(sys_modal,elim);
disp('"Modal reduction" reduced order model size:');
disp(size(sys_modred_modalred.A));

%Compare the bode plots:
P = bodeoptions ; P.PhaseWrapping = 'on' ;P.grid ='on'; P.TickLabel.FontSize = 12; P.Title.FontSize = 10;

%Collective elevon to Pitch rate
figure(1);bode(sys(9,1),'r',P);
figure(1);hold on;bode(sys_minreal(9,1),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
% set(0,'DefaultAxesFontSize',18); set(0,'DefaultLineLinewidth',2)
title('Collective elevon to Pitch rate - Frequency response');
legend('full order model','pole-zero cancellation');

figure(201); pzmap(sys(9,1),'r',sys_minreal(9,1),'b');sgrid;legend('full order model','pole-zero cancellation');

figure(2);bode(sys(9,1),'r',P);
figure(2);hold on;bode(sys_modred_residual(9,1),'k',P);
title('Collective elevon to Pitch rate - Frequency response');
legend('full order model','residulization');

figure(202); pzmap(sys(9,1),'r',sys_modred_residual(9,1),'k');sgrid;legend('full order model','residulization');

figure(3);bode(sys(9,1),'r',P);
figure(3);hold on;bode(sys_modred_trunct(9,1),'m',P);
% xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
title('Collective elevon to Pitch rate - Frequency response');
legend('full order model','truncation');

% see how the pole-zeros might be different in the original model and its
% approximations
figure(203); pzmap(sys(9,1),'r',sys_modred_trunct(9,1),'m');sgrid;legend('full order model','truncation');

figure(4);bode(sys(9,1),'r',P);
figure(4);hold on;bode(sys_modred_modalred(9,1),'g',P);
title('Collective elevon to Pitch rate - Frequency response');
legend('full order model','modal selection');

figure(204); pzmap(sys(9,1),'r',sys_modred_modalred(9,1),'g');sgrid;legend('full order model','Modal selection');
figure(205); pzmap(sys(9,1),'r');sgrid;legend('full order model');
figure(206); pzmap(sys_modred_modalred(9,1),'g');sgrid;legend('Modal selection');

figure(5);
bode(sys(9,1),'r',sys_minreal(9,1),'b',sys_modred_residual(9,1),'k',sys_modred_trunct(9,1),'m',sys_modred_modalred(9,1),'g',P);
legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
title('Collective elevon to Pitch rate - Frequency response');

figure(6);
stepplot(sys(9,1),'r',sys_minreal(9,1),'b',sys_modred_residual(9,1),'k',sys_modred_trunct(9,1),'m',sys_modred_modalred(9,1),'g',100);
legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');

figure(7);
stepplot(sys(9,1),'r--',sys_modred_residual(9,1),'k',sys_modred_trunct(9,1),'m',100);
legend('full order model','residulization','truncation');
title('Collective elevon to pitch rate - time response');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Differential elevon to roll rate
figure(31);bode(sys(8,2),'r',P);
figure(31);hold on;bode(sys_minreal(8,2),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
title('Differential elevon to roll rate - Frequency response');
legend('full order model','pole-zero cancellation');

figure(32); pzmap(sys(8,2),'r',sys_minreal(8,2),'b');sgrid;legend('full order model','pole-zero cancellation');

figure(34);bode(sys(8,2),'r',P);
figure(34);hold on;bode(sys_modred_residual(8,2),'k',P);
title('Differential elevon to roll rate - Frequency response');
legend('full order model','residulization');

figure(36); pzmap(sys(8,2),'r',sys_modred_residual(8,2),'k');sgrid;legend('full order model','residulization');

figure(38);bode(sys(8,2),'r',P);
figure(38);hold on;bode(sys_modred_trunct(8,2),'m',P);
% xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
title('Differential elevon to roll rate - Frequency response');
legend('full order model','truncation');

% see how the pole-zeros might be different in the original model and its
% approximations
figure(40); pzmap(sys(8,2),'r',sys_modred_trunct(8,2),'m');sgrid;legend('full order model','truncation');

figure(42);bode(sys(8,2),'r',P);
figure(42);hold on;bode(sys_modred_modalred(8,2),'g',P);
title('Differential elevon to roll rate - Frequency response');
legend('full order model','modal selection');

figure(44); pzmap(sys(8,2),'r',sys_modred_modalred(8,2),'g');sgrid;legend('full order model','Modal selection');

figure(46);
bode(sys(8,2),'r',sys_minreal(8,2),'b',sys_modred_residual(8,2),'k',sys_modred_trunct(8,2),'m',sys_modred_modalred(8,2),'g',P);
legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
title('Differential elevon to roll rate - Frequency response');

figure(48);
stepplot(sys(8,2),'r--',sys_modred_residual(8,2),'k',sys_modred_trunct(8,2),'m',30);
legend('full order model','residulization','truncation');
title('Differential elevon to roll rate - time response');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Differential throttle to roll rate
figure(51);bode(sys(8,4),'r',P);
figure(51);hold on;bode(sys_minreal(8,4),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
title('Differential throttle to roll rate - Frequency response');
legend('full order model','pole-zero cancellation');

figure(52); pzmap(sys(8,4),'r',sys_minreal(8,4),'b');sgrid;legend('full order model','pole-zero cancellation');

figure(54);bode(sys(8,4),'r',P);
figure(54);hold on;bode(sys_modred_residual(8,4),'k',P);
title('Differential throttle to roll rate - Frequency response');
legend('full order model','residulization');

figure(56); pzmap(sys(8,4),'r',sys_modred_residual(8,4),'k');sgrid;legend('full order model','residulization');

figure(58);bode(sys(8,4),'r',P);
figure(58);hold on;bode(sys_modred_trunct(8,4),'m',P);
% xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
title('Differential throttle to roll rate - Frequency response');
legend('full order model','truncation');

% see how the pole-zeros might be different in the original model and its
% approximations
figure(60); pzmap(sys(8,4),'r',sys_modred_trunct(8,4),'m');sgrid;legend('full order model','truncation');

figure(62);bode(sys(8,4),'r',P);
figure(62);hold on;bode(sys_modred_modalred(8,4),'g',P);
title('Differential throttle to roll rate - Frequency response');
legend('full order model','modal selection');

figure(64); pzmap(sys(8,4),'r',sys_modred_modalred(8,4),'g');sgrid;legend('full order model','Modal selection');

figure(66);
bode(sys(8,4),'r',sys_minreal(8,4),'b',sys_modred_residual(8,4),'k',sys_modred_trunct(8,4),'m',sys_modred_modalred(8,4),'g',P);
legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
title('Differential throttle to roll rate - Frequency response');

figure(68);
stepplot(sys(8,4),'r--',sys_modred_residual(8,4),'k',sys_modred_trunct(8,4),'m',50);
legend('full order model','residulization','truncation');
title('Differential throttle to roll rate - time response');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Differential throttle to yaw rate
figure(71);bode(sys(10,4),'r',P);
figure(71);hold on;bode(sys_minreal(10,4),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
title('Differential throttle to yaw rate - Frequency response');
legend('full order model','pole-zero cancellation');

figure(72); pzmap(sys(10,4),'r',sys_minreal(10,4),'b');sgrid;legend('full order model','pole-zero cancellation');

figure(74);bode(sys(10,4),'r',P);
figure(74);hold on;bode(sys_modred_residual(10,4),'k',P);
title('Differential throttle to yaw rate - Frequency response');
legend('full order model','residulization');

figure(76); pzmap(sys(10,4),'r',sys_modred_residual(10,4),'k');sgrid;legend('full order model','residulization');

figure(78);bode(sys(10,4),'r',P);
figure(78);hold on;bode(sys_modred_trunct(10,4),'m',P);
% xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
title('Differential throttle to yaw rate - Frequency response');
legend('full order model','truncation');

% see how the pole-zeros might be different in the original model and its
% approximations
figure(80); pzmap(sys(10,4),'r',sys_modred_trunct(10,4),'m');sgrid;legend('full order model','truncation');

figure(82);bode(sys(10,4),'r',P);
figure(82);hold on;bode(sys_modred_modalred(10,4),'g',P);
title('Differential throttle to yaw rate - Frequency response');
legend('full order model','modal selection');

figure(84); pzmap(sys(10,4),'r',sys_modred_modalred(10,4),'g');sgrid;legend('full order model','Modal selection');

figure(86);
bode(sys(9,4),'r',sys_minreal(10,4),'b',sys_modred_residual(10,4),'k',sys_modred_trunct(10,4),'m',sys_modred_modalred(10,4),'g',P);
legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
title('Differential throttle to yaw rate - Frequency response');

figure(88);
stepplot(sys(10,4),'r--',sys_modred_residual(10,4),'k',sys_modred_trunct(10,4),'m',20);
legend('full order model','residulization','truncation');
title('Differential throttle to yaw rate - time response');

