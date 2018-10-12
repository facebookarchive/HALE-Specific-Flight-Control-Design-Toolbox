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
%           [OptClaws]=OptimiseAquilaClaws_Discrete(LinACmodel,FltCondition,OptConfigSetupFunc, Options)
%
%	PURPOSE: Optimises Aquila Claws to meet some design stability and performance requirements
%
%	INPUTS:	The Simulink model, Optimization configuration file name, and aircraft model 
%
%	DEFAULT:
%                       default.PitchLoopTuningGoals    = ['Freq_Shaping','Step_Tracking','Margins'];          %The tuning goals
%                       default.DesiredPitchLoopBW      = 0.63;   %in rad (0.1 Hz)
%                       default.DesiredPitchLoopPM      = 60;     %in deg
%                       default.DesiredPitchLoopGM      = 8;      %in dB
%                       default.DesiredPitchLoopMaxFreq = 40;     %in rad/sec
%                       default.DesiredPitchLoopOV      = 0.15;   %percent overshoot
%                       default.optvars                 = {'AP_PitchLoop_Kp','AP_PitchLoop_Ki','AP_PitchLoop_Kd','AP_PitchLoop_Kff','AP_PitchLoop_Kq'};
%                       default.optSimulinkModel        = 'LinLongNormal_Aquila_PitchLoop';
% 
% 
% 	OUTPUTS: OptClaws	A structure with fields:
%
%
%	PRE-REQUISITES:		ASE model should be available, the standard tunbale Claws gains and coefficients should be provided
%	                    The optimization configuration function (OptConfigSetup) should be prepared to setup the optimisation
%                       objects, variables and loops 					
%
%	LIMITATIONS:		Currently many
%
%
%	DEVELOPER :			H. Bolandhemmat (Facebook Connectivity Lab)
%
%
%
function [OptClaws] = OptimizeAquilaClaws_Discrete(LinACmodel,FltCondition,OptConfigSetupFunc, Options)  %perhaps, the LinACmodel can be also defined in the Options

% ------------------------------------------
% DEFAULT INITIALISATIONS
default.ACModelDatafile             = 'Aquila_1A_6.mat';                                                               
default.ACModelDataPath             = '/Users/hbolandhemmat/Dropbox (Facebook)/Aquila/GNC/Modeling and Simulation/ASE/Aquila Matlab Simulator v1.1.3 15-December-2016';     %default folder for the aircraft ASE model data                          
default.PitchLoopTuningGoals        = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the Pitch Loop
default.ClimbRateLoopTuningGoals    = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the Climb Rate Loop
default.TASLoopTuningGoals          = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the TAS Loop
default.DesiredPitchLoopBW          = 0.63;   %in rad (0.1 Hz)
default.DesiredPitchLoopPM          = 55;     %in deg
default.DesiredPitchLoopGM          = 8;      %in dB
default.DesiredPitchLoopMaxFreq     = 40;     %in rad/sec
default.DesiredPitchLoopOV          = 0.15;   %percent overshoot
default.DesiredClimbRateLoopBW      = 0.25;   %in rad
default.DesiredClimbRateLoopPM      = 55;     %in deg
default.DesiredClimbRateLoopGM      = 8;      %in dB
default.DesiredClimbRateLoopOV      = 0.15;   %percent overshoot
default.DesiredTASLoopBW            = 0.1;   %in rad
default.DesiredTASLoopPM            = 55;     %in deg
default.DesiredTASLoopGM            = 8;      %in dB
default.DesiredTASLoopOV            = 0.15;   %percent overshoot
default.optvars                     = {'AP_PitchLoop_Kp','AP_PitchLoop_Ki','AP_PitchLoop_Kd','AP_PitchLoop_Kff','AP_PitchLoop_Kq','AP_PitchLoop_qFdbkFiltNumCoe', 'AP_PitchLoop_qFdbkFiltDenCoe'};    %Name of the vars to be optimised
default.optvarsBlk                  = {};    %Name of the Block names to be optimised
% default.optSimulinkModel          = 'LinLongNormal_Aquila_PitchLoop';
default.optBlkvarsinit              = [0.2, 0.1, 0.01, 0.1, 0.2, 0.5, 0.5] ;
default.silence                     = 0 ; 
default.optBlkvarsMin               = [0.1, 0.1, 0,   0,  0.05, 0, 0 ] ;
default.optBlkvarsMax               = [1.5, 0.9, 0.5, 10, 0.5, 1, 1] ;
default.PitchLoopOptSetting         = [1,((1e-2)*0.04)];                    % {'RandomStart','MinDecay'} ;  [10,1e-2]
default.OptOut                      = {};                                   % a structure in which the opt results are saved into
default.SkipLoadLinData             = 0;
default.WriteBlocksOptValues        = 0;                                     % If TRUE, the optimized values are written into the Simulink model blocks
default.KeepBlocksOptValues         = 0;                                     % If TRUE, the optimized values are written into the Simulink workspace to be used for the next optimization, although it wouldn't override the user defined initial condition
default.OptConfig                   = 'PitchLoop';                           % Define the loop which should be optimised
default.OptModePlot                 = 'On';                                  % If 'On' plots the open loop and closed loop system modes
default.OptviewSpec                 = 'On';
default.Ts                          = (1/25);                                % default sampling time in sec, for 25Hz sampling rate
default.OutputFileName              = 'OptOut';


%% define the constants
%Booleans
TRUE  = 1 ;
FALSE = 0 ;
%Unit conversions
Deg2Rad = pi/180;


optOptions = eval_defaults(Options,default);
extract_keyword;

OptConfigstr         = ['[Options] = ',OptConfigSetupFunc,'(',char(39),optOptions.OptConfig,char(39),');'];
eval(OptConfigstr);					% call the Optimization Configuration Function (OptConfigFile)

optOptions = eval_defaults(Options,default);
extract_keyword;

% consider the sampling time in the decay rate if Ts has been defined
if(optOptions.Ts ~= 0 )
    optOptions.PitchLoopOptSetting(2) = optOptions.PitchLoopOptSetting(2) * optOptions.Ts ;
end

%Check if the top level model is already open, 
if (strcmp(bdroot(),LinACmodel))
    close_system (LinACmodel,0);
end

%%Open the Simulink model and add the required paths
open_system(LinACmodel);

addpath(genpath(optOptions.ACModelDataPath));
Aquila  = load(optOptions.ACModelDatafile);

Ts = optOptions.Ts ;

alts   = FltCondition.alts;             %[ 0 ] ;
eass   = FltCondition.eass;             %[ 10.97 ] ;
gammas = FltCondition.gammas;           %[ 0 ] ;
phis   = FltCondition.phis;             %[ 0 ] ;

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


% Get the linearzied input and feedforward matrices at the IMU location 
[sys] = lin_outputmatrix_gen(Aquila,state_0,u_0,ialt, ieas, igamma, iphi,idelta_spoiler,1,1,0) ;
% sys_scaled = prescale(sys(9,1),{10^-3,10^2}) ;
sys_scaled = prescale(sys,{10^-3,10^2}) ;

minreal_Tol = 1e-7; % Set tolerance (increasing it, will force additional cancellations)
 
% Simplify system poles/zeros
sys_minreal = minreal(sys_scaled,minreal_Tol);
damp(sys(9,1));
damp(sys_minreal(9,1));
figure(200);bode(sys(9,1), 'b', sys_minreal(9,1),'r');
figure(202);pzplot(sys(9,1),'b',sys_minreal(9,1),'r');
% Reduce the order with creating the balance realization
Des_reduced_order = 34;
 
% Create option set for balred command
Options = balredOptions('RelTol',1e-8,'AbsTol',1e-8, 'Offset',1e-8);

% Compute reduced order approximation
sys_balred = balred(sys_scaled,Des_reduced_order,Options);

figure(203);bode(sys(9,1), 'b', sys_minreal(9,1),'r',sys_balred(9,1),'m');legend('sys','sys_minreal','sys_balred');
figure(204);pzplot(sys(9,1),'b',sys_balred(9,1),'r');
damp(sys_balred);
% find the model state space matrices at the IMU location (node 1)
LIN_AC_S_A = sys_minreal.A ;
LIN_AC_S_B = sys_minreal.B ;
LIN_AC_S_C = sys_minreal.C ;
LIN_AC_S_D = sys_minreal.D ;

% LIN_AC_S_A = sys_balred.A ;
% LIN_AC_S_B = sys_balred.B ;
% LIN_AC_S_C = sys_balred.C ;
% LIN_AC_S_D = sys_balred.D ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% I put the data here until we can manage the memory better
assignin('base','Ts',Ts);

assignin('base','LIN_AC_S_A',LIN_AC_S_A);
assignin('base','LIN_AC_S_B',LIN_AC_S_B);
assignin('base','LIN_AC_S_C',LIN_AC_S_C);
assignin('base','LIN_AC_S_D',LIN_AC_S_D);

%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract the model tunable blocks keeping the same sequence
LinACmodel_Blks         = find_system(LinACmodel) ;
n_optVars = size(optOptions.optvars,2);
n_optBlks = size(LinACmodel_Blks,1) ;
Blk_idx = 1 ;
noptVars_in_optBlks = size(find(contains(LinACmodel_Blks,optOptions.optvars)),1);    %check how many tunable parameters name exist in the model blocks
%make a temporary optOptions.optvars file:
if( ( noptVars_in_optBlks ~= size( optOptions.optvars,2) ) && any(contains(optOptions.optvars,{'NumCoe'}))  )
    optvars_temp = strrep(optOptions.optvars,'NumCoe','');
    isFiltBlkOpt = TRUE ;  %Is there a block which is optimised?
else
    optvars_temp = optOptions.optvars ;
    isFiltBlkOpt = FALSE;
end

% Get the associated blocks to the tunable parameters
%LinACmodel_Blks(contains(LinACmodel_Blks,optvars_temp))

for i_vars = 1:n_optVars
    if ( any(contains(LinACmodel_Blks,optvars_temp(i_vars))) )
        
        optOptions.optvarsBlk(Blk_idx) = LinACmodel_Blks(contains(LinACmodel_Blks,optvars_temp(i_vars)));
        Blk_idx = Blk_idx + 1;
    end
end


%% Create and configure the |slTuner| interface:
%    a) define the tunable blocks
%    b) define the analysis points

ST0     = slTuner(LinACmodel,optOptions.optvarsBlk);
ST0.Ts  = Ts;  %set the sampling rate

% Add the analysis points for the Pitch Loop
addPoint(ST0,{'theta_ref','theta_Deg','theta_err'});

% Second, adding the signals for the inner loop analysis and inner loop bandwidth
% requirements, 
% Designate as analysis points the plant inputs and outputs (control and
% measurement signals) where the stability margins are measured. 

addPoint(ST0,{'elv_Cmd_ThetaLoop','elv_Cmd_qLoop','q_Dps','q_meas','elvCmd_Deg','elv','elvCmdDeg_PitchLoop','w_d'}); %'qFiltWashed'
% addPoint(ST0,{'q_filtd','theta_filtd'});

% Second, adding the signals for the inner loop analysis and inner loop bandwidth
% requirements, 
% Designate as analysis points the plant inputs and outputs (control and
% measurement signals) where the stability margins are measured. 

addPoint(ST0,{'elv_Cmd_ThetaLoop','elv_Cmd_qLoop','q_Dps','q_meas','elvCmd_Deg','elv','elvCmdDeg_PitchLoop','w_d'}); %'qFiltWashed'
% addPoint(ST0,{'q_filtd','theta_filtd'});

% Add the analysis points for the Climb rate Loop
addPoint(ST0,{'climb_rate_cmd_mps','climb_rate_mps', 'climb_rate_error_mps', 'climb_rate_cmd_uncapped_mps','climb_rate_meas_mps' });

% Add the analysis points for the TAS Loop
addPoint(ST0,{'tas_cmd_mps','tas_mps', 'tas_error_mps', 'theta_cmd_uncapped_deg','tas_meas_mps' });



n_optBlk= size(optOptions.optvarsBlk,2);

for idx_optBlk = 1:n_optBlk
    
    OptBlksStr      = cellarray2str(optOptions.optvarsBlk(idx_optBlk));
    idx_OptBlkStr   = regexp(optOptions.optvarsBlk{idx_optBlk},'/') ;


    optOptions.optvarsBlk{idx_optBlk} = OptBlksStr(1,(idx_OptBlkStr(size(idx_OptBlkStr,2))+1):end) ;
end


idx_optVar = 1 ;

for idx_optBlk = 1:n_optBlk   %ioptvar
        
    if( isFiltBlkOpt == TRUE && contains(optOptions.optvarsBlk{idx_optBlk}, {'LLFilt'} ))
        for idx_optVar=idx_optBlk:idx_optBlk+1
            eval([optOptions.optvars{idx_optVar},'=realp(optOptions.optvars{idx_optVar}, optOptions.optvarsinit(idx_optVar));']);
            eval([optOptions.optvars{idx_optVar},'.Minimum = optOptions.optvarsMin(idx_optVar);']);
            eval([optOptions.optvars{idx_optVar},'.Maximum = optOptions.optvarsMax(idx_optVar);']);
        end
        %define a tunable transfer function (gnss)
        eval([optOptions.optvarsBlk{idx_optBlk},'=tf([1 ',optOptions.optvars{idx_optBlk},'],[1 ',optOptions.optvars{idx_optBlk+1},'],ST0.Ts);'])
        %put the tunable tf (gnss) into the block with the same name
        eval(['setBlockParam(ST0,', char(39), sprintf(optOptions.optvarsBlk{idx_optBlk}),char(39), ',', optOptions.optvarsBlk{idx_optBlk},')']);

    else
        eval([optOptions.optvars{idx_optVar},'=realp(optOptions.optvars{idx_optVar}, optOptions.optvarsinit(idx_optVar));']);
        eval([optOptions.optvars{idx_optVar},'.Minimum = optOptions.optvarsMin(idx_optVar);']);
        eval([optOptions.optvars{idx_optVar},'.Maximum = optOptions.optvarsMax(idx_optVar);']);

        eval(['setBlockParam(ST0,', char(39), sprintf(optOptions.optvars{idx_optBlk}),char(39), ',', optOptions.optvars{idx_optBlk},')']);
        
        idx_optVar = idx_optVar + 1;
    end

end

%% 
% Display a summary of the |slTuner| interface configuration in the command
% window, just for now, after debugging should be removed

if (optOptions.silence == FALSE )
    ST0 
end

%%
% Extract the preliminary closed loop response from 'theta_ref' to 'theta'
% responses between the reference signals and |'theta'| 
T_theta_CL = getIOTransfer(ST0,{'theta_ref'},{'theta_Deg'},{'theta_cmd_uncapped_deg'});
figure(7);hold on;stepplot(T_theta_CL,50,'r');
% G_theta  = tf(T_theta_CL)    %Do not use this command when T_theta_CL is a
% discrete model, it removes some of the modes unwantedly, This is a Matlab
% bug.
P = bodeoptions ; P.PhaseWrapping = 'on' ;P.grid ='on';
%G_theta  = minreal(G_theta) ;
figure(17);hold on;bode(T_theta_CL,'r', P); 
if( contains(optOptions.OptModePlot, 'On') )
    damp(T_theta_CL); P_T_theta_CL=pole(T_theta_CL); Z_T_theta_CL=zero(T_theta_CL); figure(171); pzmap(T_theta_CL);zgrid;legend('T-theta');
end
% T_theta_CL_notch = getIOTransfer(ST0,{'theta_ref'},{'theta_filtd'});
% figure(17);hold on;bode(T_theta_CL_notch,'b', P); 
% figure(172); pzmap(T_theta_CL_notch);zgrid;legend('T-theta-notch');
% Extract the open loop response from 'elv' to 'q'
%add the {'elv'} as the fourth input to break the loop at 'elv';
%otherwise the PI controller would be in the loop as well
T_q_OL     = getIOTransfer(ST0,{'elvCmd_Deg'},{'q_Dps'},{'elvCmd_Deg'});   %open loop TF
if(contains(optOptions.OptModePlot, 'On'))
    P1 = pzoptions; P1.Grid = 'on';  %P1 = pzoptions;P1.Xlim=[-30,10];P1.Ylim=[-30,30]; P1.Grid = 'on';
    damp(T_q_OL); P_G_q_OL = pole(T_q_OL); Z_G_q_OL =zero(T_q_OL);figure(1811);pzplot(T_q_OL,P1);legend('Open Loop Modes');  %figure(181); pzmap(T_q_OL)
    figure(8);stepplot(T_q_OL,35,'r')
    figure(18); hold on; bode(T_q_OL, 'b', T_q_OL_minreal,'r', P);

end

%% Extract the 'Augmented' open loop response from 'elv_Cmd_ThetaLoop' to 'q'
T_q_InnerLoop     = getIOTransfer(ST0,{'elv_Cmd_ThetaLoop'},{'q_Dps'},{'elv_Cmd_ThetaLoop'});   
figure(8);hold on; stepplot(T_q_InnerLoop,50,'b')
figure(18);hold on; bode(T_q_InnerLoop,'k', P);
if(contains(optOptions.OptModePlot, 'On'))
    damp(T_q_InnerLoop); P_G_q_InnerLoop = pole(T_q_InnerLoop);Z_G_q_InnerLoop = zero(T_q_InnerLoop);figure(181); pzmap(T_q_InnerLoop);zgrid;legend('Open loop with the rate feedback');
end
% T_q_InnerLoop_notch     = getIOTransfer(ST0,{'elv_Cmd_ThetaLoop'},{'q_filtd'},{'elv_Cmd_ThetaLoop'});   
% figure(18);hold on; bode(T_q_InnerLoop_notch,'m', P);

%To get the pitch/theta loop Loop gain (open-loop transfer L=C*G measured location 'elv-Cmd-ThetaLoop':
T_theta_LG  = getLoopTransfer(ST0,'elv_Cmd_ThetaLoop',-1,{'theta_cmd_uncapped_deg'});  %'elv-Cmd-ThetaLoop' 
% figure(17);hold on; bode(G_theta_LG,'r',P);
figure(179); bode(T_theta_LG,P);grid on;
figure(179);hold on;bode(T_q_OL,'r',P);
figure(179);hold on;bode(T_q_InnerLoop,'k', P);
figure(179);hold on;bode(sys(9,1),'b',P);legend('Loop Gain', 'Open Loop','Augmented Inner loop','Contunious model');
%% Get the sensitivity function measured at the input of the plant
addPoint(ST0,'elvCmd_Deg')
elv_S0 = getSensitivity(ST0,'elvCmd_Deg',{'theta_cmd_uncapped_deg'});
figure(11);hold on; bodeplot(elv_S0,P);grid on;

%for a SISO system, sensitivity should be identical:
q_S0  = getSensitivity(ST0,'q_meas',{'theta_cmd_uncapped_deg'});
figure(15);hold on;bodeplot(q_S0,P);grid on;

if( contains(optOptions.OptConfig,'ClimbRateLoop') )
    % Extract the preliminary closed loop response from 'climb_rate_cmd_mps' to 'climb_rate_mps'
    T_climb_rate_CL = getIOTransfer(ST0,{'climb_rate_cmd_mps'},{'climb_rate_mps'},{'climb_rate_cmd_uncapped_mps'});
    figure(70);hold on;stepplot(T_climb_rate_CL,50,'r');
    figure(71);bode(T_climb_rate_CL,'r',P);grid on;

    %To get the Climb rate loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
    loop_gain_tas_loop_input  = getLoopTransfer(ST0,'theta_cmd_uncapped_deg',-1,{'climb_rate_cmd_uncapped_mps'}); 
    figure(73); bode(loop_gain_tas_loop_input,'r',P);grid on;
     
    %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
    loop_gain_tas_loop_output  = getLoopTransfer(ST0,'climb_rate_mps',-1,{'climb_rate_cmd_uncapped_mps'}); 
    figure(73); hold on;bode(loop_gain_tas_loop_output,'k.-',P);grid on;

    %compare the inner loop and outer loop frequency responses
    figure(72);bode(T_climb_rate_CL,'r',P);grid on;hold on;bode(T_theta_CL,'b',P);legend('Closed loop Climb rate loop','Closed loop Pitch/rate attitude loop');

    
end

if( contains(optOptions.OptConfig,'TASLoop') )
    % Extract the preliminary closed loop response from 'tas_cmd_mps' to 'tas_mps'
    T_tas_CL = getIOTransfer(ST0,{'tas_cmd_mps'},{'tas_mps'});
    figure(90);hold on;stepplot(T_tas_CL,50,'r');
    figure(91);bode(T_tas_CL,'r',P);grid on;

    %To get the tas loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
    loop_gain_tas_loop_input  = getLoopTransfer(ST0,'climb_rate_cmd_uncapped_mps',-1); 
    figure(92); bode(loop_gain_tas_loop_input,'r',P);grid on;
     
    %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
    loop_gain_tas_loop_output  = getLoopTransfer(ST0,'tas_mps',-1); 
    figure(92); hold on;bode(loop_gain_tas_loop_output,'k.-',P);grid on;

    %compare the inner loop and outer loop frequency responses for the tas loop
    figure(93);bode(T_tas_CL,'k',P);hold on;bode(T_climb_rate_CL,'r',P);hold on;bode(T_theta_CL,'b',P);grid on;legend('Closed loop TAS', 'Closed loop Climb rate','Closed loop Pitch attitude');

    
end


%% Capture the design requirements:

SetOptObjectives;

%% Capture the Optimisation settings:
PitchLoopOptSetting = systuneOptions('RandomStart',optOptions.PitchLoopOptSetting(1),'MinDecay',optOptions.PitchLoopOptSetting(2));

%% call the systune and put the tuned parameters in the new structure: ST1
[ST1,fSoft,~,Tinfo] = systune(ST0,Objectives_All,PitchLoopOptSetting);

if (optOptions.silence == FALSE )
    showTunable(ST1)
end


optOptions.OptOut = ST1 ;
OptClaws          = ST1 ;

if( optOptions.KeepBlocksOptValues == TRUE )
    
% Write the new results, if a "replace" flag is on to write the designed
% parameters into the parameter callback function to be used as initial conditions for the 
% next run
    hws = get_param(bdroot, 'modelworkspace');
%     hws.DataSource = 'MAT-File';              %Should define the AquilaDiscretePitchClawsParameters.m file 
    hws.FileName = 'AquilaDiscretePitchClawsParameters';
   for idx_optVar=1:size(optOptions.optvars,2)
       eval(['hws.assignin(',optOptions.optvars{idx_optVar},'char(39)',optOptions.optvars{idx_optVar},'.Value);'])
   end
    
%     hws.assignin('wn', 4);
%     hws.assignin('zeta', 0.8);
    hws.saveToSource;
    hws.reload;
end

if( contains(optOptions.OptConfig,'PitchLoop') )
    figure('Position',[100,100,900,474]);
    viewSpec(Objectives_PitchLoop,ST1,Tinfo);
end

if( contains(optOptions.OptConfig,'ClimbRateLoop') )
    figure('Position',[100,100,900,474]);
    viewSpec(Objectives_ClimbRateLoop,ST1,Tinfo);
end

if( contains(optOptions.OptConfig,'TASLoop') )
    figure('Position',[100,100,900,474]);
    viewSpec(Objectives_TASLoop,ST1,Tinfo);
end


%%

%write the tuned gains/parameters back to see the impact
if ( optOptions.WriteBlocksOptValues == TRUE )
    writeBlockValue(ST1);
end
if( isFiltBlkOpt == TRUE && contains(optOptions.optvarsBlk{idx_optBlk}, {'LLFilt'} ) )
    [q_FdbkLLFilt] = tf(getBlockValue(ST1,'AP_PitchLoop_qFdbkLLFilt'))
    figure(21);bode(q_FdbkLLFilt);
    figure(22);pzmap(q_FdbkLLFilt);
end

T_theta_CL_new = getIOTransfer(ST1,{'theta_ref'},{'theta_Deg'},{'theta_cmd_uncapped_deg'});
figure(7);hold on;step(T_theta_CL_new,50);
isstable(T_theta_CL_new);

% T_theta_notch_new = getIOTransfer(ST1,{'theta_ref'},{'theta_filtd'});
% figure(7);hold on;step(T_theta_notch_new,50);


figure(17);hold on; bode(T_theta_CL_new,'g',P);grid on;legend('Pre design','post design')
if(contains(optOptions.OptModePlot, 'On'))
    damp(T_theta_new); P_G_theta_CL_new = pole(T_theta_CL_new);Z_G_theta_CL_new = zero(T_theta_CL_new);
    
    % damp(T_theta_notch_new);

    % figure(172); pzmap(T_theta_notch_new);zgrid;legend('T-theta-notch');
    figure(173); pzmap(T_theta_CL_new);zgrid;legend('T-theta-new');

    figure(170);pzmap(T_theta_CL, 'r',T_theta_CL_new,'g');zgrid;legend('T-theta','T-theta-new');
    %To get the pitch/theta loop Loop gain (open-loop transfer L=C*G measured at the location 'elv-Cmd-ThetaLoop':

end


%get the closed loop transfer function function from theta_ref to elvCmd_Deg
T_elv_new = getIOTransfer(ST1,{'theta_ref'},{'elvCmdDeg_PitchLoop'},{'theta_cmd_uncapped_deg'});

if(contains(optOptions.OptModePlot, 'On'))
    damp(T_elv_new);
end
figure(173); pzmap(T_elv_new);zgrid;legend('T-elv-new');

T_err_new = getIOTransfer(ST1,{'theta_ref'},{'theta_err'},{'theta_cmd_uncapped_deg'});
if(contains(optOptions.OptModePlot, 'On'))
    damp(T_err_new);
    figure(174); pzmap(T_err_new);zgrid;legend('T-err-new');
end


T_theta_disturbnce_new = getIOTransfer(ST1,{'w_d'},{'theta_Deg'},{'theta_cmd_uncapped_deg'});
if(contains(optOptions.OptModePlot, 'On'))
    damp(T_theta_disturbnce_new);
    figure(175); pzmap(T_theta_disturbnce_new);zgrid;legend('T-theta-disturbnce-new');
end

LG_theta_new  = getLoopTransfer(ST1,'elv_Cmd_ThetaLoop',-1,{'theta_cmd_uncapped_deg'});  %'elv-Cmd-ThetaLoop' 
figure(179);hold on; bode(LG_theta_new,P,'m');

%% Extract the 'Augmented' open loop response from 'elv_Cmd_ThetaLoop' to 'q'
T_q_InnerLoop_new     = getIOTransfer(ST1,{'elv_Cmd_ThetaLoop'},{'q_Dps'},{'elv_Cmd_ThetaLoop'});   
figure(179);hold on; bode(T_q_InnerLoop_new,'g', P);legend('Loop Gain', 'Open Loop','Augmented Inner loop','Contunious model','Designed Loop gain','Designed Inner Loop');

figure(180); bode(T_theta_LG,P);grid on;
figure(180);hold on;bode(T_q_OL,'r',P);
figure(180);hold on;bode(T_q_InnerLoop,'k', P);legend('Loop Gain', 'Open Loop','Augmented Inner loop');

figure(187);pzplot(T_q_OL,'r',T_q_InnerLoop_new,'m',T_theta_CL_new,'b');legend('Open Loop','Inner Loop', 'closed Loop (pitch)');
figure(188);pzplot(T_q_OL,'r',T_q_InnerLoop_new,'m');legend('Open Loop','Inner Loop');
figure(189);pzplot(T_q_InnerLoop_new,'m',T_theta_CL_new,'b');legend('Inner Loop', 'closed Loop (pitch)');


OL_IN_SM = getLoopTransfer(ST1,'elvCmd_Deg',-1,{'theta_cmd_uncapped_deg'}); % negative-feedback loop transfer broken at input
figure(172);margin(OL_IN_SM);
OL_OUT_SM = getLoopTransfer(ST1,'q_Dps',-1,{'theta_cmd_uncapped_deg'});     % negative-feedback loop transfer broken at output
figure(182);hold on;margin(OL_OUT_SM);
grid;
xlim([1e-2,1e2]);

%To get the pitch/theta loop Loop gain (open-loop transfer L=G*C measured at the location 'theta_Deg':
LG_theta_new  = getLoopTransfer(ST1,'theta_Deg',-1,{'theta_cmd_uncapped_deg'});  %'elv-Cmd-ThetaLoop' 
figure(1707);hold on; bode(LG_theta_new,P,'m');grid on;

%Compare the new design with the initial settings for the climb rate loop:
if( contains(optOptions.OptConfig,'ClimbRateLoop') )
    % Extract the preliminary closed loop response from 'climb_rate_cmd_mps' to 'climb_rate_mps'
    T_climb_rate_CL_new = getIOTransfer(ST1,{'climb_rate_cmd_mps'},{'climb_rate_mps'},{'climb_rate_cmd_uncapped_mps'});
    figure(70);hold on;stepplot(T_climb_rate_CL_new,50,'b');legend('Pre design','Post design');
    figure(71);hold on;bode(T_climb_rate_CL_new,'b',P);grid on;legend('Pre design','Post design');

    %compare the inner loop and outer loop frequency responses
    figure(72);hold on;bode(T_climb_rate_CL_new,'r-.',P);grid on;hold on;bode(T_theta_CL_new,'b-.',P);legend('Closed loop Climb rate','Closed loop Pitch attitude');

    %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
    loop_gain_tas_loop_output_new  = getLoopTransfer(ST0,'climb_rate_mps',-1,{'climb_rate_cmd_uncapped_mps'}); 
    figure(73); hold on;bode(loop_gain_tas_loop_output_new,'g.-',P);grid on;

    %To get the Climb rate loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
    loop_gain_tas_loop_input_new  = getLoopTransfer(ST1,'theta_cmd_uncapped_deg',-1,{'climb_rate_cmd_uncapped_mps'}); 
    figure(73); hold on;bode(loop_gain_tas_loop_input_new,'b',P);grid on;

    
end

if( contains(optOptions.OptConfig,'TASLoop') )
    % Extract the newly designed closed loop response from 'tas_cmd_mps' to 'tas_mps'
    T_tas_CL_new = getIOTransfer(ST1,{'tas_cmd_mps'},{'tas_mps'});
    figure(90);hold on;stepplot(T_tas_CL_new,50,'b');legend('Pre design','Post design');
    figure(91);hold on;bode(T_tas_CL_new,'b',P);grid on;legend('Pre design','Post design');

    %To get the tas loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
    loop_gain_tas_loop_input_new  = getLoopTransfer(ST1,'climb_rate_cmd_uncapped_mps',-1); 
    figure(92); hold on;bode(loop_gain_tas_loop_input_new,'m',P);grid on;
     
    %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
    loop_gain_tas_loop_output_new  = getLoopTransfer(ST1,'tas_mps',-1); 
    figure(92); hold on;bode(loop_gain_tas_loop_output_new,'g.-',P);grid on;

    %compare the inner loop and outer loop frequency responses for the tas
    %loop after the design
    figure(93);hold on;bode(T_tas_CL,'k-.',P);hold on;bode(T_climb_rate_CL,'r-.',P);hold on;bode(T_theta_CL,'b-.',P);grid on;legend('Closed loop TAS', 'Closed loop Climb rate','Closed loop Pitch attitude');
    figure(94);hold on;bode(T_tas_CL_new,'k',P);hold on;bode(T_climb_rate_CL_new,'r',P);hold on;bode(T_theta_CL_new,'b',P);grid on;legend('Closed loop TAS', 'Closed loop Climb rate','Closed loop Pitch attitude');
    
end

% % Save the optimization output file to use for the next optimisation call	
save(optOptions.OutputFileName,'OptClaws')
