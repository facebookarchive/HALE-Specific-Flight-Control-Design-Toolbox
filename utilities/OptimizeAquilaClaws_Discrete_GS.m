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
%           [OptClaws]=OptimiseAquilaClaws_Discrete_GS(LinACmodel,FltCondition,OptConfigSetupFunc, Options)
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
%                       OptimizeAquilaClaws_Discrete_GS(LinACmodel,FltCondition,OptConfigSetupFunc, Options)
function [OptClaws] = OptimizeAquilaClaws_Discrete_GS(LinACmodel,FltCondition,OptConfigSetupFunc, Options)  %perhaps, the LinACmodel can be also defined in the Options

% ------------------------------------------
% DEFAULT INITIALISATIONS
default.ACModelName                 = 'Aquila_OL_Model_15_May_2018_08_16_19.mat';          %the *.mat file containing the aircraft ASE model
default.UseCustomModel              = 0;                                                   %Choose whether to use aswing ROM model
default.CustomModelName             = '';                                                  %Name of aswing ROM file
default.ClimbRateLoopTuningGoals    = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the Climb Rate Loop
default.TASLoopTuningGoals          = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the TAS Loop
default.PwrLoopTuningGoals          = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the Pwr Loop
default.AltLoopTuningGoals          = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the Alt Loop
default.YawRateLoopTuningGoals      = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning gaols for the Yaw rate loop
default.RollLoopTuningGoals         = {'Freq_Shaping','Step_Tracking','Margins'};          %The tuning goals for the Roll/rate Loop
default.FlexLoopTuningGoals         = {'Margins'};
default.GLALoopTuningGoals          = {'Margins'};
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
default.DesiredTASLoopOV            = 0.05;   %percent overshoot
default.DesiredPwrLoopBW            = 0.2;    %in rad
default.DesiredPwrLoopPM            = 55;     %in deg
default.DesiredPwrLoopGM            = 8;      %in dB
default.DesiredPwrLoopOV            = 0.05;   %percent overshoot
default.DesiredAltLoopBW            = 0.1;    %in rad
default.DesiredAltLoopPM            = 55;     %in deg
default.DesiredAltLoopGM            = 8;      %in dB
default.DesiredAltLoopOV            = 0.1;    %percent overshoot
default.DesiredRollLoopBW           = 0.1;    %in rad
default.DesiredRollLoopPM           = 55;     %in deg
default.DesiredRollLoopGM           = 8;      %in dB
default.DesiredRollLoopMaxFreq      =100;     %in rad
default.DesiredRollLoopOV           = 0.1;    %percent overshoot
default.DesiredGndTrkLoopBW         = 0.1;    %in rad
default.DesiredGndTrkLoopPM         = 55;     %in deg
default.DesiredGndTrkLoopGM         = 8;      %in dB
default.DesiredGndTrkLoopOV         = 0.1;    %percent overshoot
default.DesiredYawRateLoopBW        = 0.1;    %in rad
default.DesiredYawRateLoopPM        = 55;     %in deg
default.DesiredYawRateLoopGM        = 8;      %in dB
default.DesiredYawRateLoopOV        = 0.1;    %percent overshoot
default.DesiredFlexLoopGM           = 6;      %in dB
default.DesiredFlexLoopPM           = 45;     %in deg
default.DesiredGLALoopGM            = 6;      %in dB
default.DesiredGLALoopPM            = 45;     %in deg
default.optvars                     = {'AP_PitchLoop_Kp','AP_PitchLoop_Ki','AP_PitchLoop_Kd','AP_PitchLoop_Kff','AP_PitchLoop_Kq','AP_PitchLoop_qFdbkFiltNumCoe', 'AP_PitchLoop_qFdbkFiltDenCoe'};    %Name of the vars to be optimised
default.optvarsBlk                  = {};    %Name of the Block names to be optimised
default.optvarsinit                 = [0.3,    0.12,   0.05,  -0.5,  -0.9,  -0.5,   -0.9] ;
default.silence                     = 0 ; 
default.optvarsMin                  = [0.05,   0.05,   0.05,  -1,    -1,    -1,     -1 ] ;
default.optvarsMax                  = [1.5,    0.9,    0.5,    0 ,    0,     0,     0] ;
default.PitchLoopOptSetting         = [1,((1e-2)*0.04)];                     % {'RandomStart','MinDecay'} ;  [10,1e-2]
default.OptOut                      = {};                                    % a structure in which the opt results are saved into
default.SkipLoadLinData             = 0;
default.WriteBlocksOptValues        = 1;                                     % If TRUE, the optimized values are written into the Simulink model blocks
default.KeepBlocksOptValues         = 0;                                     % If TRUE, the optimized values are written into the Simulink workspace to be used for the next optimization, although it wouldn't override the user defined initial condition
default.Optimize4MultipleFlts       = 1;                                     % If TRUE (1), performs the optimization for multiple flight scenarios (make sure multiple flight conditions have been provided)
default.OptConfig                   = 'PitchLoop';                           % Define the loop which should be optimised
default.OptModePlot                 = 'Off';                                 % If 'On' plots the open loop and closed loop system modes
default.OptviewSpec                 = 'On';
default.SISOMargin                  = 'Off';
default.MIMOMargin                  = 'Off';                                 % If 'On', also measures the MIMO (disk) margins
default.Write2PptFile               =  0;                                    % If TRUE (1), will write all the results into a *.ppt file
default.Write2XcelSheet             =  0;                                    % If TRUE (1), will write all the results into a *.xlsx sheet
default.Ts                          = (1/25);
default.OutputFileName              = 'OptOut.mat';                          %should be a mat file which contains the opt results, the control system performance measures as well as the flight conditions for which the control system has been designed.
default.OutputXcelFileName          = 'ClawsOptOut.csv';                    %should be a mat file which contains the opt results, the control system performance measures as well as the flight conditions for which the control system has been designed.
default.ReadFltCndFrmCSVFile        = 0 ;                                   %If TRUE, will read the input flight conditions from an input csv data file
default.InputXcelFileName           = 'FltCndInput.csv';                    %should be a mat file which contains the opt results, the control system performance measures as well as the flight conditions for which the control system has been designed.
default.UsePrevOptResultAsIC        = 0;                                    % If this is set to zero, the code uses the standard pre-defined initial conditions for the control parameters/coeffs. However if it set to 1, then it uses the last round of tuned parameters stored in 'OptOut.mat'
default.getAllCrossovers            = 1;                                    %Some loops often have multiple places where they cross the 0db line. If this is set to 1 we get all of them in our output file and table and if its set to 0, we just get the last one.
default.PlotOpenLoopFltDyn          = 1 ;                                   %Plot Open Loop Flight Dynamic bodes, step responses and pole/zero
default.ShowOpenLoopFltDyn          = 1 ;                                   %Shows the aircraft open loop properties, location of sensors and servos and the mode shapes for the single flight condition (@Trim) 
default.UseFullOrderModel           = 1 ;                                   %Uses full order model for single flight condition control laws design (@Trim)
default.UseTruncatedReducedOrder    = 0 ;                                   %Uses reduced order model for single flight condition control laws design - The order is reduced by truncation
default.UseResidulizedReducedOrder  = 0 ;                                   %Uses reduced order model for single flight condition control laws design - The order is reduced by residulization 
default.FlexControlActive           = 1 ;                                   %If TRUE (1), designs control laws to suppress lightly damped flexible modes (higher altitude, marginal speeds)   
default.FlexControlInputIndex       = 1 ;                                   %Index of the most effective control surface input for flexible mode control - based on the analysis in Module 2:AquilaFltDynAnalysis
default.FlexControlOutputIndex      = 16 ;                                  %Index of the most effective measurement output for the flexible mode control - based on the analysis in Module 2:AquilaFltDynAnalysis         
default.GLAControlActive            = 1 ;                                   %If TRUE (1), designs control laws to alleviate loads on the wing and body of the aircraft due to gust  


%% define the constants
%Booleans
TRUE  = 1 ;
FALSE = 0 ;

%Unit conversions
Deg2Rad = pi/180;


optOptions = get_default_options(Options,default);
set_options;

OptConfigstr         = ['[Options] = ',OptConfigSetupFunc,'(',char(39),optOptions.OptConfig,char(39),',',(num2str(optOptions.Optimize4MultipleFlts)),');'];
eval(OptConfigstr);					% call the Optimization Configuration Function (OptConfigFile)

optOptions = get_default_options(Options,default);
set_options;

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

if(optOptions.UseCustomModel==0)
    %This part is Aquila model specific - must be modified for generalized model
    % Restore the paths to the "Aircraft" object in the model folders
    load(optOptions.ACModelName, 'sim_folder') ;
    % Check to see if the path exists:
    if(exist(sim_folder,'dir') == 7) % check if the folder exists
        addpath(genpath(sim_folder));
    else
        error(['Path to Aircraft model object in ' optOptions.ACModelName '.sim_folder does not exist. (' sim_folder ')']);
    end
    load(optOptions.ACModelName);
end


alts   = FltCondition.alts;             
eass   = FltCondition.eass;             
gammas = FltCondition.gammas;           
phis   = FltCondition.phis;             

%% Check if optimisation tool is run over mutiple flight conditions
if (optOptions.Optimize4MultipleFlts ~= TRUE )
    
    getAquilaModel ;
    
    %Ask user to continue:
    disp('.........');
    disp('about to configure the closed loop time/frequency responses with the initial control laws gains/coefficients provided (see the OptConfigSetup.m script for details on the initial conditions)');   
    disp('.........');
    
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
  
    if (optOptions.silence == FALSE )
        disp('******************************************************************');
        disp('List of the blocks to be tuned during the control laws design process with their detailed addresses within the simulink model ................');
        disp('******************************************************************');
        disp(ST0.TunedBlocks)
        disp('Controller sampling time is :');
        disp(ST0.Ts); 
        disp('Simulink model used containing the aircraft model and the flight control laws structure is :');
        disp(ST0.Model)
    end
    
    %% Define loops analysis points
    SetAnalysisPts ;

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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Ask user to continue:
    disp('.........');
    reply = 'Y';
%     reply = input('closed loop time/frequency responses with the initial control laws gains/coefficients are plotted next .... press J to jump over it, otherwise press any key to continue:    ','s');
    if isempty(reply)
      reply = 'Y';
    end

    if reply ~= 'J'
        % Extract the preliminary closed loop response from 'theta_ref' to 'theta'
        % responses between the reference signals and |'theta'| 
        T_theta_CL = getIOTransfer(ST0,{'theta_ref'},{'theta_Deg'},{'theta_cmd_uncapped_deg','thrCmd_Norm'});
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
        % Notch filter effect analsis
        % T_theta_CL_notch = getIOTransfer(ST0,{'theta_ref'},{'theta_filtd'});
        % figure(17);hold on;bode(T_theta_CL_notch,'b', P); 
        % figure(172); pzmap(T_theta_CL_notch);zgrid;legend('T-theta-notch');
        % Extract the open loop response from 'elv' to 'q'
        %add the {'elv'} as the fourth input to break the loop at 'elv';
        %otherwise the PI controller would be in the loop as well
        T_q_OL     =  getIOTransfer(ST0,{'elvCmd_Deg'},{'q_Dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg','vertGust_mps'});   %open loop TF

       if(contains(optOptions.OptModePlot, 'On'))
            P1 = pzoptions; P1.Grid = 'on';  %P1 = pzoptions;P1.Xlim=[-30,10];P1.Ylim=[-30,30]; P1.Grid = 'on';
            damp(T_q_OL); P_G_q_OL = pole(T_q_OL); Z_G_q_OL =zero(T_q_OL);figure(1811);pzplot(T_q_OL,P1);legend('Open Loop Modes');  %figure(181); pzmap(T_q_OL)
            figure(8);stepplot(T_q_OL,35,'r')
            figure(18); hold on; bode(T_q_OL, 'b', T_q_OL_minreal,'r', P);

            T_thetaIngtd_OL     = getIOTransfer(ST0,{'elvCmd_Deg'},{'theta_intgd_deg'},{'elvCmd_Deg','thrCmd_Norm','splCmd_Deg'});   %open loop TF
            T_theta_OL          = getIOTransfer(ST0,{'elvCmd_Deg'},{'theta_Deg'},{'elvCmd_Deg','thrCmd_Norm','splCmd_Deg'});   %open loop TF

            figure(3318); hold on; bode(T_q_OL, 'b', T_thetaIngtd_OL, 'r.-', T_theta_OL, 'k.-', P);

        end

        %% Extract the 'Augmented' open loop response from 'elv_Cmd_ThetaLoop' to 'q'
        T_q_InnerLoop     = getIOTransfer(ST0,{'elv_Cmd_ThetaLoop'},{'q_Dps'},{'elv_Cmd_ThetaLoop','thrCmd_Norm','splCmd_Deg'});   
        figure(8);hold on; stepplot(T_q_InnerLoop,50,'b')
        figure(18);hold on; bode(T_q_InnerLoop,'k', P);
        if(contains(optOptions.OptModePlot, 'On'))
            damp(T_q_InnerLoop); P_G_q_InnerLoop = pole(T_q_InnerLoop);Z_G_q_InnerLoop = zero(T_q_InnerLoop);figure(181); pzmap(T_q_InnerLoop);zgrid;legend('Open loop with the rate feedback');
        end

        % T_q_InnerLoop_notch     = getIOTransfer(ST0,{'elv_Cmd_ThetaLoop'},{'q_filtd'},{'elv_Cmd_ThetaLoop'});
        % figure(18);hold on; bode(T_q_InnerLoop_notch,'m', P);

        %To get the pitch/theta loop Loop gain (open-loop transfer L=C*G measured location 'elv-Cmd-ThetaLoop':
        T_theta_LG  = getLoopTransfer(ST0,'elv_Cmd_ThetaLoop',-1,{'theta_cmd_uncapped_deg','thrCmd_Norm'});  %'elv-Cmd-ThetaLoop' 
        % figure(17);hold on; bode(G_theta_LG,'r',P);
        figure(179); bode(T_theta_LG,P);grid on;
        figure(179);hold on;bode(T_q_OL,'r',P);
        figure(179);hold on;bode(T_q_InnerLoop,'k', P);
        % FIXME This is the only plot that plots from the Matlab sys
        % instead of from Simulink.  Is that intentional?
%         figure(179);hold on;bode(sys('q_radps','elvCmd_rad'),'b.',P);legend('Loop Gain', 'Open Loop','Augmented Inner loop','Contunious model');
        %% Get the sensitivity function measured at the input of the plant
        % This is an input disturbance, so disturbance enters at the plant input, the input sensitivity function (elv_S0) is the closed loop
        % transfer function from the disturbance at elvCmd_Deg to elvCmd_Deg
        
        elv_S0 = getSensitivity(ST0,'elvCmd_Deg');
        figure(11);hold on; bodeplot(elv_S0,P);grid on;

        %Sensitivity at the output measurements
        q_S0  = getSensitivity(ST0,'q_meas');
        figure(15);hold on;bodeplot(q_S0,P);grid on;
        
        %get the open/closed loop dynamics from vertical gust to pitch rate
        T_q_wg_OL   = getIOTransfer(ST0,{'vertGust_mps'},{'q_meas'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});                  %open loop TF
        figure(3);bode(T_q_wg_OL,'r',P);
        T_q_wg_CL   = getIOTransfer(ST0,{'vertGust_mps'},{'q_meas'});                                                                                           %closed loop TF
        figure(3);hold on;bode(T_q_wg_CL,'b',P);
        
        T_theta_wg_CL   = getIOTransfer(ST0,{'vertGust_mps'},{'theta_Deg'});                                                                                    %closed loop TF
        figure(4);hold on;bode(T_theta_wg_CL,'b',P);
        
        T_nzIMU_wg_OL   = getIOTransfer(ST0,{'vertGust_mps'},{'a_z_IMU_mpss'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});                                                                            %closed loop TF
        figure(5);bode(T_nzIMU_wg_OL,'r',P);
        T_nzIMU_wg_CL   = getIOTransfer(ST0,{'vertGust_mps'},{'a_z_IMU_mpss'});                                                                                  %closed loop TF
        figure(5);hold on;bode(T_nzIMU_wg_CL,'b',P);title('Nz response to vertical gust');legend('GLA loop open','GLA loop closed');
        
        %Time domain responses from gust
        figure(6);stepplot(-T_nzIMU_wg_OL,50,'r');
        figure(6);hold on;stepplot(-T_nzIMU_wg_CL,50,'b');grid;xlabel('time(sec)');ylabel('normal acceleration (m/s/s)');legend('Open Loop','Closed Loop');title('Acceleration step response from gust')
        
        if( contains(optOptions.OptConfig,'ClimbRateLoop') )
            % Extract the preliminary closed loop response from 'climb_rate_cmd_mps' to 'climb_rate_mps'
            T_climb_rate_CL = getIOTransfer(ST0,{'climb_rate_cmd_spdloop_mps'},{'climb_rate_mps'},{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'});
            figure(70);hold on;stepplot(T_climb_rate_CL,50,'r');
            figure(71);bode(T_climb_rate_CL,'r',P);grid on;

            %To get the Climb rate loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
            loop_gain_tas_loop_input  = getLoopTransfer(ST0,'theta_cmd_uncapped_deg',-1,{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'}); 
            figure(73); bode(loop_gain_tas_loop_input,'r',P);grid on;

            %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
            loop_gain_tas_loop_output  = getLoopTransfer(ST0,'climb_rate_mps',-1,{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'}); 
            figure(73); hold on;bode(loop_gain_tas_loop_output,'k.-',P);grid on;

            %compare the inner loop and outer loop frequency responses
            figure(72);bode(T_climb_rate_CL,'r',P);grid on;hold on;bode(T_theta_CL,'b',P);legend('Closed loop Climb rate loop','Closed loop Pitch/rate attitude loop');

        end

        if( contains(optOptions.OptConfig,'TASLoop') )
            % Extract the preliminary closed loop response from 'tas_cmd_mps' to 'tas_mps'
            T_tas_CL = getIOTransfer(ST0,{'tas_cmd_mps'},{'tas_mps'},{'thrCmd_Norm'});
            figure(90);hold on;stepplot(T_tas_CL,150,'r');
            figure(91);bode(T_tas_CL,'r',P);grid on;

            %To get the tas loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
            loop_gain_tas_loop_input  = getLoopTransfer(ST0,'climb_rate_cmd_spdloop_uncapped_mps',-1,{'thrCmd_Norm'}); 
            figure(92); bode(loop_gain_tas_loop_input,'r',P);grid on;

            %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
            loop_gain_tas_loop_output  = getLoopTransfer(ST0,'tas_mps',-1,{'thrCmd_Norm'}); 
            figure(92); hold on;bode(loop_gain_tas_loop_output,'k.-',P);grid on;

            %compare the inner loop and outer loop frequency responses for the tas loop
            figure(93);bode(T_tas_CL,'k',P);hold on;bode(T_climb_rate_CL,'r',P);hold on;bode(T_theta_CL,'b',P);grid on;legend('Closed loop TAS', 'Closed loop Climb rate','Closed loop Pitch attitude');

        end
        
        
        if( contains(optOptions.OptConfig,'AltLoop') )
            % Extract the preliminary closed loop response from 'climb_rate_cmd_altloop_mps' to 'climb_rate_mps'
            T_alt_CL = getIOTransfer(ST0,{'alt_cmd_m'},{'presAlt_meas_m'});
            figure(460);hold on;stepplot(T_alt_CL,150,'r');
            figure(462);bode(T_alt_CL,'r',P);grid on;

            %To get the pwr loop Loop gain (open-loop transfer L=C*G measured location 'climb_rate_cmd_altloop_mps' (broken @ input signal):
            loop_gain_alt_loop_input  = getLoopTransfer(ST0,'climb_rate_cmd_altloop_mps',-1,{'theta_cmd_uncapped_deg'}); 
            figure(464); hold on;bode(loop_gain_alt_loop_input,'b',P);grid on;legend('alt loop gain measured at input (climb_rate_cmd_altloop_mpsm)');

            %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'presAlt_meas_m' (broken @ output signal):
            loop_gain_alt_loop_output  = getLoopTransfer(ST0,'presAlt_meas_m',-1,{'theta_cmd_uncapped_deg'}); 
            figure(466); hold on;bode(loop_gain_alt_loop_output,'k.-',P);grid on;
            
            %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'presAlt_meas_m' (broken @ output signal):
            loop_gain_alt_loop  = getLoopTransfer(ST0,'presAlt_meas_m',-1); 
            figure(468); hold on;bode(loop_gain_alt_loop,'k.-',P);grid on;

        end
        

        if( contains(optOptions.OptConfig,'RollLoop') )
            %p is the x body component of the angular velocity
            T_p_Ail_OL   = getIOTransfer(ST0,{'ailCmd_Deg'},{'p_dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});           %lateral open loop TF
            T_p_Rdr_OL   = getIOTransfer(ST0,{'diffThrCmd_Norm'},{'p_dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});      %lateral open loop TF

            T_aos_Ail_OL = getIOTransfer(ST0,{'ailCmd_Deg'},{'aos_deg'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});           %lateral open loop TF


            T_phi_OL           = getIOTransfer(ST0,{'ailCmd_Deg'},{'phi_deg'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});          %open loop TF
            T_phiIntgd_OL      = getIOTransfer(ST0,{'ailCmd_Deg'},{'phi_intgd_deg'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});    %open loop TF

            if(contains(optOptions.OptModePlot, 'On'))
                P1 = pzoptions; P1.Grid = 'on';
                damp( T_p_Ail_OL); P_G_p_OL = pole( T_p_Ail_OL); Z_G_p_OL =zero( T_p_Ail_OL);
    %             figure(2812);pzplot(T_phiRate_OL,'b', T_phiRate_OL_DiffThrCLsd,'r',P1);legend('Open Loop Modes for lateral channel');  %figure(181); pzmap(T_q_OL)
                figure(2812);pzplot(T_p_Ail_OL,'b',P1);legend('Open Loop Modes for lateral channel');
                figure(401);stepplot( T_p_Ail_OL,250,'r');
                figure(401);hold on;stepplot( T_phi_OL,250,'b');legend('step Aileron to body roll rate (P_dps)  - OL','step Aileron to roll attitude - OL');grid on;

    %             figure(402); hold on; bode( T_phiRate_OL, 'b', T_phiRate_OL_DiffThrCLsd,'r',T_phi_OL,'b.-',T_phi_OL_DiffThrCLsd,'r.-',P);
                figure(402); hold on; bode( T_phi_OL,'r', T_p_Ail_OL, 'b', T_p_Rdr_OL,'g',T_aos_Ail_OL,'k',P);legend('OL aileron to roll attitude','OL aileron to roll rate','OL Rdr to Roll Rate','OL aileron to sideslip angle');
                figure(403); hold on; bode( T_p_Ail_OL, 'b', T_phiIntgd_OL, 'r.-', T_phi_OL, 'k.-',P);legend('OL aileron to roll attitude','OL aileron to integrated roll','OL aileron to roll');

                figure(404); hold on; bode( T_aos_Ail_OL,'k',P);legend('OL aileron to sideslip angle');grid on;

            end

            % Extract the preliminary closed loop response from 'phi_ref' to 'phi_meas_deg'
            T_phi_CL             = getIOTransfer(ST0,{'phi_ref'},{'phi_meas_deg'},{'phi_cmd_deg'});
            T_phi_CL_DirOpen     = getIOTransfer(ST0,{'phi_ref'},{'phi_meas_deg'},{'phi_cmd_deg','diffThrCmd_Norm'});

            figure(410);hold on;stepplot(T_phi_CL,150,'r');
            figure(410);hold on;stepplot(T_phi_CL_DirOpen,150,'b');legend('CL phi_ref to phi- Directional loop closed','CL phi_ref to phi- Directional loop open');grid on;

            figure(414);bode(T_phi_CL,'r',T_phi_CL_DirOpen,'b',P);grid on;legend('CL phi_ref to phi- Directional loop closed','CL phi_ref to phi- Directional loop open');

            ail_S0 = getSensitivity(ST0,'ailCmd_Deg',{'phi_cmd_deg'});
            figure(413);hold on; bodeplot(ail_S0,'k',P);grid on;

            phi_S0 = getSensitivity(ST0,'phi_meas_deg',{'phi_cmd_deg'});
            figure(413);hold on; bodeplot(phi_S0,'r',P);grid on;

            %To get the roll loop Loop gain (open-loop transfer L=G*C measured location 'phi_meas_deg' (broken @ output signal):
            loop_gain_phi_loop_output  = getLoopTransfer(ST0,'Ail',-1,{'phi_cmd_deg'}); 
            figure(416); hold on;bode(loop_gain_phi_loop_output,'g',P);grid on;
            
            %To get the roll loop Loop gain (open-loop transfer L=G*C measured location 'phi_meas_deg' (broken @ output signal):
            loop_gain_phi_loop_output  = getLoopTransfer(ST0,'phi_meas_deg',-1,{'phi_cmd_deg'}); 
            figure(416); hold on;bode(loop_gain_phi_loop_output,'k',P);grid on; 
           
            %To get the roll loop Loop gain (open-loop transfer L=G*C measured location 'rollRate_meas_dps' (broken @ output signal) when the directional loop is open:

            loop_gain_phi_loop_output_DirOpen  = getLoopTransfer(ST0,'phi_meas_deg',-1,{'phi_cmd_deg','diffThrCmd_Norm'}); 
            figure(416); hold on;bode(loop_gain_phi_loop_output_DirOpen,'b',P);grid on;legend('bank attitude loop gain - measured at Ail command input','bank attitude loop gain - Directional loop closed','bank attitude loop gain - Directional loop open');

            T_r_Rdr_OL   = getIOTransfer(ST0,{'diffThrCmd_Norm'},{'r_dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});           %lateral open loop TF
            figure(417); bode(T_r_Rdr_OL,'b',P);grid on;

            T_p_Rdr_OL   = getIOTransfer(ST0,{'diffThrCmd_Norm'},{'p_dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});           %lateral open loop TF
            figure(417); hold on;bode(T_p_Rdr_OL,'r',P);grid on;legend('Rdr to yaw rate (OL)','Rdr to roll rate (OL)');

            T_p_r_OL = T_p_Rdr_OL / T_r_Rdr_OL ;
            figure(417); hold on;bode(T_p_r_OL,'g',P);grid on;legend('Rdr to yaw rate (OL)','Rdr to roll rate (OL)','p/r (OL)');

    %         K_roll_Intgr = tf(0.01,[1, 0]);
    %         L_rollLoop = series(K_roll_Intgr,T_p_r_OL);
    %         figure(417); hold on;bode(L_rollLoop,'k',P);grid on;legend('Rdr to yaw rate (OL)','Rdr to roll rate (OL)','p/r (OL)','Roll Loop & Integrator');

        end

       %% preliminary loop information for the GndTrk Loop
        if( contains(optOptions.OptConfig,'GndTrkLoop') )

            T_gndTrk_CL = getIOTransfer(ST0,{'gndTrk_cmd_rad'},{'gndTrk_meas_rad'});
            figure(418);hold on;stepplot(T_gndTrk_CL,500,'r');
            figure(419);bode(T_gndTrk_CL,'r',P);grid on;

            %To get the GndTrk loop Loop gain (open-loop transfer L=C*G measured location 'phi_cmd_deg' (broken @ input signal):
            loop_gain_gndTrk_loop_input  = getLoopTransfer(ST0,'phi_cmd_deg',-1); 
            figure(420); bode(loop_gain_gndTrk_loop_input,'r',P);grid on;legend('gndTrk loop gain measured at the loop input (phi_cmd_deg)');

            %To get the roll loop Loop gain (open-loop transfer L=G*C measured location 'gndTrk_meas_rad' (broken @ output signal):
            loop_gain_gndTrk_loop_output  = getLoopTransfer(ST0,'gndTrk_meas_rad',-1); 
            figure(421); hold on;bode(loop_gain_gndTrk_loop_output,'k',P);grid on;legend('gndTrk loop gain measured at the loop output (gndTrk_meas_rad)');

            figure(420); hold on;bode(loop_gain_gndTrk_loop_output,'k.',P);grid on;legend('gndTrk loop gain at the input and output');
            
            %To get the roll loop Loop gain (open-loop transfer L=G*C measured location 'gndTrk_meas_rad' (broken @ output signal):
            loop_gain_gndTrk_loop_output_DirOpen  = getLoopTransfer(ST0,'gndTrk_meas_rad',-1,{'diffThrCmd_Norm'}); 
            figure(421); hold on;bode(loop_gain_gndTrk_loop_output_DirOpen,'b',P);grid on;legend('gndTrk loop gain - Directional loop closed','gndTrk loop gain - Directional loop open');


        end

        if( contains(optOptions.OptConfig,'YawRateLoop') )

            T_yawRate_Rdr_OL   = getIOTransfer(ST0,{'diffThrCmd_Norm'},{'r_dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm','splCmd_Deg'});      %directional open loop TF

            if(contains(optOptions.OptModePlot, 'On'))
                P1 = pzoptions; P1.Grid = 'on';
                damp( T_yawRate_Rdr_OL ); P_G_r_OL = pole(T_yawRate_Rdr_OL ); Z_G_r_OL =zero( T_yawRate_Rdr_OL );

                figure(2912);pzplot(T_yawRate_Rdr_OL,'b',P1);legend('Open Loop Modes for directional channel');
                figure(422);stepplot( T_yawRate_Rdr_OL,45,'r')
                figure(423); hold on; bode( T_yawRate_Rdr_OL, 'b',P);

            end

            % Extract the preliminary closed loop response from 'yawRate_ref_dps' to 'yawRate_meas_dps'
            T_yawRateFrmRollLoop_CL = getIOTransfer(ST0,{'yawRate_ref_dps'},{'yawRate_meas_dps'},{'phi_cmd_deg'});  %phi_cmd_filtAdv'
            
            figure(426);hold on;stepplot( T_yawRateFrmRollLoop_CL,500,'r.-');
            figure(427);hold on;bode( T_yawRateFrmRollLoop_CL,'r.-',P);grid on;

            T_yawRate_Frm_phiCmd_CL = getIOTransfer(ST0,{'phi_cmd_deg'},{'yawRate_meas_dps'});
            figure(426);hold on;stepplot( T_yawRate_Frm_phiCmd_CL,50,'b');legend('CL response from yaw rate cmd to yaw rate','CL response from bank attitude cmd to yaw rate');grid on;
            figure(427);hold on;bode( T_yawRate_Frm_phiCmd_CL,'b',P); legend('CL bode from yaw rate cmd to yaw rate','CL bode from bank attitude cmd to yaw rate');grid on;

            T_yawRate_Frm_Cmd_CL   = getIOTransfer(ST0,{'yawRate_ref_dps'},{'yawRate_meas_dps'},{'yawRate_ref_uncapped_dps'});
            figure(426);hold on;stepplot(T_yawRate_Frm_Cmd_CL,150,'k');legend('CL bode from yaw rate cmd to yaw rate','CL bode from bank attitude cmd to yaw rate');grid on;
            figure(427);hold on;bode(T_yawRate_Frm_Cmd_CL,'k',P);grid on;legend('CL bode only for the yaw rate/Rdr loop');grid on;


            %To get the yaw rate loop Loop gain (open-loop transfer L=C*G measured location 'diffThrCmd_Norm' (broken @ input signal):
            loop_gain_yawRate_loop_input  = getLoopTransfer(ST0,'diffThrCmd_Norm',-1,{'phiErrAdv_cmd'}); 
            figure(429); bode(loop_gain_yawRate_loop_input,'r',P);grid on;

            %To get the yaw rate loop Loop gain (open-loop transfer L=C*G measured location 'yawRate_meas_dps' (broken @ output signal) - GndTRk loop is open @:'phi_cmd_deg'
            loop_gain_yawRate_loop_output  = getLoopTransfer(ST0,'yawRate_meas_dps',-1,{'phi_cmd_deg'}); 
            figure(430); bode(loop_gain_yawRate_loop_output,'r',P);grid on;

            %To get the yaw rate loop Loop gain (open-loop transfer L=C*G measured location 'yawRate_meas_dps' (broken @ output signal) - GndTRk loop is open @:'phi_cmd_deg'
            loop_gain_yawRateFrmYawCmd_loop_output  = getLoopTransfer(ST0,'yawRate_meas_dps',-1,{'yawRate_ref_uncapped_dps'}); 
            figure(430); bode(loop_gain_yawRateFrmYawCmd_loop_output,'r',P);grid on;
            
            %To get the yaw rate loop Loop gain (open-loop transfer L=C*G measured location 'yawRate_meas_dps' (broken @ output signal) - :
            loop_gain_yawRateFrmPhiCmdAdv_loop_output  = getLoopTransfer(ST0,'yawRate_meas_dps',-1,{'phiErrAdv_cmd'}); 
            figure(430); hold on; bode(loop_gain_yawRateFrmPhiCmdAdv_loop_output,'g',P);

            %To get the yaw rate from roll attitude command loop Loop gain (open-loop transfer L=C*G measured location 'diffThrCmd_Norm' (broken @ input signal):
            loop_gain_yawRateFrmPhiCmd_loop_input  = getLoopTransfer(ST0,'diffThrCmd_Norm',-1); 
            figure(431); bode(loop_gain_yawRateFrmPhiCmd_loop_input,'r',P);grid on;

            %To get the yaw rate from roll attitude command loop Loop gain (open-loop transfer L=C*G measured location 'yawRate_meas_dps' (broken @ output signal):
            loop_gain_yawRateFrmPhiCmd_loop_output  = getLoopTransfer(ST0,'yawRate_meas_dps',-1); 
            figure(430); hold on;bode(loop_gain_yawRateFrmPhiCmd_loop_output,'b',P);grid on;legend('Loop gain at the yaw rate meas - disconnected from the Ground track loop','Loop gain at the yaw rate meas - disconnected from the roll attitude loop','Loop gain at the yaw rate meas - all loops are closed');

        end
        
        
    end
    
    %% Capture the design requirements:
    disp('.........');
    disp('setting and plotting the control laws design objectives for various loops (see inside the simulink model for control laws structure)'); 
    disp('The control laws design time and frequency domain requirements are defined in OptConfigSetup.m script');
    SetOptObjectives;

    %% Capture the Optimisation settings:
    ClawsOptSetting = systuneOptions('RandomStart',optOptions.PitchLoopOptSetting(1),'MinDecay',optOptions.PitchLoopOptSetting(2),'UseParallel',TRUE,'Display','iter');
 
     %Ask user to continue:
    disp('.........');
    reply = 'Y';
%     reply = input('about to start the control laws design process (executation time could be long depending on the PitchLoopOptSetting) .... press C to cancel the execution, otherwise press any key to continue:    ','s');
    if isempty(reply)
      reply = 'Y';
    end
    
    if reply == 'C'
        error(' Aborting the executation as per user request ... ');
    end
    
    %% call the systune and put the tuned parameters in the new structure: ST1
    [ST1,fSoft,~,Tinfo] = systune(ST0,Objectives_All,ClawsOptSetting);
      
    if (optOptions.silence == FALSE )
        disp('******************************************************************');
        disp('final cost function for each opt objective: ');
        disp('******************************************************************');
        disp(fSoft); 
    end
    
    if (optOptions.silence == FALSE )
        showTunable(ST1)
    end


    %optOptions.OptOut = ST1 ;

    

    %% Put the optimisation results in the m file to be used in the next model callback, could be also used as the initial condition for the next optimisation run
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

    %% Compare the designed system frequency/time performance to that of the
    %% desired responses/requirements
     if( contains(optOptions.OptConfig,'PitchLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_PitchLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'PitchLoop');
    end

    if( contains(optOptions.OptConfig,'ClimbRateLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_ClimbRateLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'ClimbRateLoop');
    end

    if( contains(optOptions.OptConfig,'TASLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_TASLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'TASLoop');
    end
    
    if( contains(optOptions.OptConfig,'PwrLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_PwrLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'PwrLoop');
    end

    if( contains(optOptions.OptConfig,'AltLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_AltLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'AltLoop');
    end
    
    if( contains(optOptions.OptConfig,'RollLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_RollLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'RollLoop');
    end

    if( contains(optOptions.OptConfig,'GndTrkLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_GndTrkLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'GndTrkLoop');
    end

    if( contains(optOptions.OptConfig,'YawRateLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_YawRateLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'YawRateLoop');
    end

     if( contains(optOptions.OptConfig,'FlexLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_FlexLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'FlexLoop');
    end

    %%
          
    %write the tuned gains/parameters back to see the impact
    if ( optOptions.WriteBlocksOptValues == TRUE )
        writeBlockValue(ST1);
    end
    if( isFiltBlkOpt == TRUE && contains(optOptions.optvarsBlk{idx_optBlk}, {'LLFilt'} ) )
        [q_FdbkLLFilt] = tf(getBlockValue(ST1,'AP_PitchLoop_qFdbkLLFilt')) ;
        figure(21);bode(q_FdbkLLFilt);
        figure(22);pzmap(q_FdbkLLFilt);
    end

      %Ask user to continue:
    disp('.........');
    reply = 'Y';
%     reply = input('about to perform additional closed-loop (post-) analysis .... press J to jump over it, otherwise press any key to continue:    ','s');
    if isempty(reply)
      reply = 'Y';
    end
    
    if reply ~= 'J'

        T_theta_CL_new = getIOTransfer(ST1,{'theta_ref'},{'theta_Deg'},{'theta_cmd_uncapped_deg','thrCmd_Norm'});
        figure(7);hold on;step(T_theta_CL_new,50);
        isstable(T_theta_CL_new);

        % T_theta_notch_new = getIOTransfer(ST1,{'theta_ref'},{'theta_filtd'});
        % figure(7);hold on;step(T_theta_notch_new,50);

        T_theta_CL = getIOTransfer(ST0,{'theta_ref'},{'theta_Deg'},{'theta_cmd_uncapped_deg','thrCmd_Norm'});


        P = bodeoptions ; P.PhaseWrapping = 'on' ;P.grid ='on';
        
        figure(17);hold on; bode(T_theta_CL,'r', P);
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
        T_elv_new = getIOTransfer(ST1,{'theta_ref'},{'elvCmdDeg_PitchLoop'},{'theta_cmd_uncapped_deg','thrCmd_Norm'});

        if(contains(optOptions.OptModePlot, 'On'))
            damp(T_elv_new);
        end
        figure(173); pzmap(T_elv_new);zgrid;legend('T-elv-new');

        T_err_new = getIOTransfer(ST1,{'theta_ref'},{'theta_err'},{'theta_cmd_uncapped_deg','thrCmd_Norm'});
        if(contains(optOptions.OptModePlot, 'On'))
            damp(T_err_new);
            figure(174); pzmap(T_err_new);zgrid;legend('T-err-new');
        end


        T_theta_disturbnce_new = getIOTransfer(ST1,{'w_d'},{'theta_Deg'},{'theta_cmd_uncapped_deg','thrCmd_Norm'});
        if(contains(optOptions.OptModePlot, 'On'))
            damp(T_theta_disturbnce_new);
            figure(175); pzmap(T_theta_disturbnce_new);zgrid;legend('T-theta-disturbnce-new');
        end

        LG_theta_new  = getLoopTransfer(ST1,'elv_Cmd_ThetaLoop',-1,{'theta_cmd_uncapped_deg','thrCmd_Norm'});  %'elv-Cmd-ThetaLoop' 
        figure(179);hold on; bode(LG_theta_new,P,'m');

        %% Extract the 'Augmented' open loop response from 'elv_Cmd_ThetaLoop' to 'q'
        T_q_InnerLoop_new     = getIOTransfer(ST1,{'elv_Cmd_ThetaLoop'},{'q_Dps'},{'elv_Cmd_ThetaLoop','thrCmd_Norm'});   
        figure(179);hold on; bode(T_q_InnerLoop_new,'g', P);legend('Loop Gain', 'Open Loop','Augmented Inner loop','Contunious model','Designed Loop gain','Designed Inner Loop');

        %To get the pitch/theta loop Loop gain (open-loop transfer L=C*G measured location 'elv-Cmd-ThetaLoop':
        T_theta_LG      = getLoopTransfer(ST0,'elv_Cmd_ThetaLoop',-1,{'theta_cmd_uncapped_deg','thrCmd_Norm'});  %'elv-Cmd-ThetaLoop'

        T_q_OL          = getIOTransfer(ST0,{'elvCmd_Deg'},{'q_Dps'},{'elvCmd_Deg','ailCmd_Deg','thrCmd_Norm','diffThrCmd_Norm'});   %open loop TF

        T_q_InnerLoop   = getIOTransfer(ST0,{'elv_Cmd_ThetaLoop'},{'q_Dps'},{'elv_Cmd_ThetaLoop','thrCmd_Norm'});   

        figure(180); bode(T_theta_LG,P);grid on;
        figure(180);hold on;bode(T_q_OL,'r',P);
        figure(180);hold on;bode(T_q_InnerLoop,'k', P);legend('Loop Gain', 'Open Loop','Augmented Inner loop');

        figure(187);pzplot(T_q_OL,'r',T_q_InnerLoop_new,'m',T_theta_CL_new,'b');legend('Open Loop','Inner Loop', 'closed Loop (pitch)');
        figure(188);pzplot(T_q_OL,'r',T_q_InnerLoop_new,'m');legend('Open Loop','Inner Loop');
        figure(189);pzplot(T_q_InnerLoop_new,'m',T_theta_CL_new,'b');legend('Inner Loop', 'closed Loop (pitch)');


        OL_IN_SM = getLoopTransfer(ST1,'elvCmd_Deg',-1,{'theta_cmd_uncapped_deg','thrCmd_Norm'}); % negative-feedback loop transfer broken at input
        figure(172);margin(OL_IN_SM);
        OL_OUT_SM = getLoopTransfer(ST1,'q_Dps',-1,{'theta_cmd_uncapped_deg','thrCmd_Norm'});     % negative-feedback loop transfer broken at output
        figure(182);hold on;margin(OL_OUT_SM);
        grid;
        xlim([1e-2,1e2]);

        %To get the pitch/theta loop Loop gain (open-loop transfer L=G*C measured at the location 'theta_Deg':
        LG_theta_new  = getLoopTransfer(ST1,'theta_Deg',-1,{'theta_cmd_uncapped_deg','thrCmd_Norm'});  %'elv-Cmd-ThetaLoop' 
        figure(1707);hold on; bode(LG_theta_new,P,'m');grid on;

        
        %get the open/closed loop dynamics from vertical gust to pitch rate
        T_q_wg_CL   = getIOTransfer(ST0,{'vertGust_mps'},{'q_meas'});                                                                            %closed loop TF
        figure(3);hold on;bode(T_q_wg_CL,'b',P);
        
        T_q_wg_CL_new   = getIOTransfer(ST1,{'vertGust_mps'},{'q_meas'});                                                                            %closed loop TF
        figure(3);hold on;bode(T_q_wg_CL_new,'k',P);legend('Pre design','Post design');

        %get the open/closed loop dynamics from vertical gust to normal
        %acceleration
        T_nzIMU_wg_CL   = getIOTransfer(ST0,{'vertGust_mps'},{'a_z_IMU_mpss'});                                                                            %closed loop TF
        figure(5);hold on;bode(T_nzIMU_wg_CL,'b',P);

        T_nzIMU_wg_CL_new   = getIOTransfer(ST1,{'vertGust_mps'},{'a_z_IMU_mpss'});                                                                            %closed loop TF
        figure(5);hold on;bode(T_nzIMU_wg_CL_new,'k',P);legend('Pre design','Post design');

               
        
        %Compare the new design with the initial settings for the climb rate loop:
        if( contains(optOptions.OptConfig,'ClimbRateLoop') )
            % Extract the preliminary closed loop response from 'climb_rate_cmd_mps' to 'climb_rate_mps'
            T_climb_rate_CL_new = getIOTransfer(ST1,{'climb_rate_cmd_spdloop_mps'},{'climb_rate_mps'},{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'});
            figure(70);hold on;stepplot(T_climb_rate_CL_new,50,'b');legend('Pre design','Post design');
            figure(71);hold on;bode(T_climb_rate_CL_new,'b',P);grid on;legend('Pre design','Post design');

            %compare the inner loop and outer loop frequency responses
            figure(72);hold on;bode(T_climb_rate_CL_new,'r-.',P);grid on;hold on;bode(T_theta_CL_new,'b-.',P);legend('Closed loop Climb rate','Closed loop Pitch attitude');

            %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
            loop_gain_tas_loop_output_new  = getLoopTransfer(ST0,'climb_rate_mps',-1,{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'}); 
            figure(73); hold on;bode(loop_gain_tas_loop_output_new,'g.-',P);grid on;

            %To get the Climb rate loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
            loop_gain_tas_loop_input_new  = getLoopTransfer(ST1,'theta_cmd_uncapped_deg',-1,{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'}); 
            figure(73); hold on;bode(loop_gain_tas_loop_input_new,'b',P);grid on;


        end

        if( contains(optOptions.OptConfig,'TASLoop') )
            % Extract the newly designed closed loop response from 'tas_cmd_mps' to 'tas_mps'
            T_tas_CL_new = getIOTransfer(ST1,{'tas_cmd_mps'},{'tas_mps'},{'thrCmd_Norm'});
            figure(90);hold on;stepplot(T_tas_CL_new,50,'b');legend('Pre design','Post design');
            figure(91);hold on;bode(T_tas_CL_new,'b',P);grid on;legend('Pre design','Post design');

            %To get the tas loop Loop gain (open-loop transfer L=C*G measured location 'theta_cmd_uncapped_deg' (broken @ input signal):
            loop_gain_tas_loop_input_new  = getLoopTransfer(ST1,'climb_rate_cmd_spdloop_uncapped_mps',-1,{'thrCmd_Norm'}); 
            figure(92); hold on;bode(loop_gain_tas_loop_input_new,'m',P);grid on;

            %To get the Climb rate loop Loop gain (open-loop transfer L=G*C measured location 'theta_cmd_uncapped_deg' (broken @ output signal):
            loop_gain_tas_loop_output_new  = getLoopTransfer(ST1,'tas_mps',-1,{'thrCmd_Norm'}); 
            figure(92); hold on;bode(loop_gain_tas_loop_output_new,'g.-',P);grid on;

            %compare the inner loop and outer loop frequency responses for the tas
            %loop after the design
            T_tas_CL        = getIOTransfer(ST0,{'tas_cmd_mps'},{'tas_mps'},{'thrCmd_Norm'});
            T_climb_rate_CL = getIOTransfer(ST0,{'climb_rate_cmd_spdloop_mps'},{'climb_rate_mps'},{'climb_rate_cmd_spdloop_uncapped_mps','thrCmd_Norm'});

            figure(93);hold on;bode(T_tas_CL,'k-.',P);hold on;bode(T_climb_rate_CL,'r-.',P);hold on;bode(T_theta_CL,'b-.',P);grid on;legend('Closed loop TAS', 'Closed loop Climb rate','Closed loop Pitch attitude');
            figure(94);hold on;bode(T_tas_CL_new,'k',P);hold on;bode(T_climb_rate_CL_new,'r',P);hold on;bode(T_theta_CL_new,'b',P);grid on;legend('Closed loop TAS', 'Closed loop Climb rate','Closed loop Pitch attitude');

            T_tas_CL     = getIOTransfer(ST0,{'tas_cmd_mps'},{'tas_mps'});
            T_tas_CL_new = getIOTransfer(ST1,{'tas_cmd_mps'},{'tas_mps'});
            figure(96);stepplot(T_tas_CL,50,'b');hold on;stepplot(T_tas_CL_new,50,'r');legend('Pre design','Post design');
            figure(97);bode(T_tas_CL,'b',P);hold on;bode(T_tas_CL_new,'r',P);grid on;legend('Pre design','Post design');

            
        end
    end

else
    
    %get the Aquila aircraft linearized model at various flight conditions
    %and put them in an n by m matrix where n is number of pressure altitudes
    %and m is number of dynamic pressures at which the aircraft nonlinear
    %ASE model has been linearized.
    getAquilaModel;

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


    TuningGrid = struct('pres_alt_m',pres_alt_m,'DynPres_Pa',DynPres_Pa);
    ShapeFcn   = @(pres_alt_m,DynPres_Pa) [pres_alt_m,DynPres_Pa,pres_alt_m*DynPres_Pa];
    
    for idx_optVar = 1:n_optVars
            
        eval([optOptions.optvars{idx_optVar},'=tunableSurface(',char(39), sprintf(optOptions.optvars{idx_optVar}), char(39), ',', 'optOptions.optvarsinit(idx_optVar),TuningGrid, ShapeFcn );']);
                   
     end


    LinACmodel_Blks         = find_system(LinACmodel) ;
    for i=1:n_optBlks
        if ( contains(LinACmodel_Blks(i),'Aquila_Lin_Model') ) 
            Aquila_LIN_Model_Name = LinACmodel_Blks(i);
            break;
        end
    end


    BlockSubs = struct('Name',Aquila_LIN_Model_Name,'Value',Aquila_LIN);

    ST0       = slTuner(LinACmodel,optOptions.optvarsBlk, BlockSubs);

    ST0.Ts    = optOptions.Ts;  %set the sampling rate

    %%Define loops analysis points
    SetAnalysisPts ;
    
    n_optBlk= size(optOptions.optvarsBlk,2);

    for idx_optBlk = 1:n_optBlk

        OptBlksStr      = cellarray2str(optOptions.optvarsBlk(idx_optBlk));
        idx_OptBlkStr   = regexp(optOptions.optvarsBlk{idx_optBlk},'/') ;


        optOptions.optvarsBlk{idx_optBlk} = OptBlksStr(1,(idx_OptBlkStr(size(idx_OptBlkStr,2))+1):end) ;
    end

     for idx_optVar = 1:n_optVars
            
        eval(['ST0.setBlockParam(', char(39), sprintf(optOptions.optvars{idx_optVar}),char(39), ',', optOptions.optvars{idx_optVar},')']);
                
     end

    %% 
    % Display a summary of the |slTuner| interface configuration in the command
    % window, just for now, after debugging should be removed

    if (optOptions.silence == FALSE )
        display(ST0); 
    end

    %% Capture the design requirements and create the LoopStruct_list for post-processing purposes:

    SetOptObjectives;
    
    disp('.........');
    disp('about to start the control gains tuning... - press any key to continue')
    pause;
    %% Capture the Optimisation settings:
    ClawsOptSetting = systuneOptions('RandomStart',optOptions.PitchLoopOptSetting(1),'MinDecay',optOptions.PitchLoopOptSetting(2),'UseParallel',TRUE,'Display','iter');

    %% call the systune and put the tuned parameters in the new structure: ST1
    [ST1,fSoft,~,Tinfo] = systune(ST0,Objectives_All,ClawsOptSetting);

    
    if (optOptions.silence == FALSE )
        disp('******************************************************************');
        disp('final cost function for each opt objective: ');
        disp('the optimisation final cost functions magnitude are (normalized):'); 
        display(fSoft);
        disp('******************************************************************');
    end
    
    if (optOptions.silence == FALSE )
        showTunable(ST1)
    end


    if( contains(optOptions.OptConfig,'PitchLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_PitchLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'PitchLoop');
    end

    if( contains(optOptions.OptConfig,'ClimbRateLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_ClimbRateLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'ClimbRateLoop');
    end

    if( contains(optOptions.OptConfig,'TASLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_TASLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'TASLoop');
    end
    
    if( contains(optOptions.OptConfig,'PwrLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_PwrLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'PwrLoop');
    end

    if( contains(optOptions.OptConfig,'AltLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_AltLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'AltLoop');
    end
    
    if( contains(optOptions.OptConfig,'RollLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_RollLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'RollLoop');
    end

    if( contains(optOptions.OptConfig,'GndTrkLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_GndTrkLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'GndTrkLoop');
    end

    if( contains(optOptions.OptConfig,'YawRateLoop') )
        figure('Position',[100,100,900,474]);
        viewSpec(Objectives_YawRateLoop,ST1,Tinfo);
        handle=gcf;
        LoopStruct_list=AddViewSpecStruct(handle,FltCondition,...
                                          LoopStruct_list,'YawRateLoop');
    end

    % Get tuned gain surfaces, and Plot them

    
    for idx_optBlk = 1:n_optBlk
        TGS_idx_optBlk  = ST1.getBlockParam(optOptions.optvarsBlk{idx_optBlk}) ;
        %Reserve the figure numbers 20 to 90 for plotting the tunable parameters 
        figure((20+idx_optBlk));viewSurf(TGS_idx_optBlk), title(optOptions.optvarsBlk{idx_optBlk}) ;
        handle=gcf;
        LoopStruct_list=AddGainSurfaceStructs(handle,LoopStruct_list);
    end
    
    
    %%

    %write the tuned gains/parameters back to see the impact
    if ( optOptions.WriteBlocksOptValues == TRUE )
        writeBlockValue(ST1);
    end
        
        
        
end

disp('.........');
disp('saving the design data into files including the spreadsheet defined by OutputXcelFileName in the OptConfigSetup.m script');
disp('.........');

% should include all the detailed opt results, the control system performance measures as well as the flight conditions for which the control system has been designed
OptClaws=TunedConditionGenerator(ST1,FltCondition,LoopStruct_list,optOptions);
OptClaws.PopulateTunedConditions;
%OptClaws=Systems;

%Save the optimization output file to use for the next optimisation call	
save(optOptions.OutputFileName,'OptClaws');

if ( optOptions.Write2XcelSheet  == TRUE )
    %Write the control system performance measures along with the corresponding flight conditions to the excel sheet
    OptClaws.WriteCharCSV(optOptions.OutputXcelFileName);

end

%Plot the speed comparison with the flexible mode on
if ( optOptions.FlexControlActive == TRUE && ~strcmp(bdroot(),LinACmodel) )
    %add the trim eas to the linearized data 
    airspeed_vector_cmd  = eass(1) + Flex_Control_Spd_Compare.Data(:,1);
    airspeed_vector_meas = eass(1) + Flex_Control_Spd_Compare.Data(:,2);

    figure;plot(Flex_Control_Spd_Compare.Time,airspeed_vector_cmd,'b');hold on;plot(Flex_Control_Spd_Compare.Time,airspeed_vector_meas,'r');xlabel('time (sec)'); ylabel('eas (m/s)');title(['Flex Mode Control Performance at ', num2str(eass(1)), ' m/s and ',num2str(alts(1)),' m']);legend('eas command (mps)','eas (mps)');grid on;
    figure;plot(Flex_Control_Spd_Compare.Time,Flex_Control_Spd_Compare.Data(:,4),'r');hold on;plot(Flex_Control_Spd_Compare.Time,Flex_Control_Spd_Compare.Data(:,6),'b');xlabel('time (sec)'); ylabel('nz (m/s/s) , Elv_flex_deg');title(['Flex Mode Control Performance at ', num2str(eass(1)), ' m/s and ',num2str(alts(1)),' m']);legend('nz offcenter blended (m/s/s)','Flex control elevator command (deg)');grid on;
    figure;plot(Flex_Control_Spd_Compare.Time,Flex_Control_Spd_Compare.Data(:,3),'g');xlabel('time (sec)'); ylabel('alt (m)');title(['Flex Mode Control Performance at ', num2str(eass(1)), ' m/s and ',num2str(alts(1)),' m']);legend('alt (m)');grid on;

    %load the data file first
    load('Flex_Control_Spd_Compare.mat')
    %add the trim eas to the linearized data 
    airspeed_vector_cmd  = eass(1) + Flex_Control_Spd_Compare_dataset.Data(:,1);
    airspeed_vector_meas = eass(1) + Flex_Control_Spd_Compare_dataset.Data(:,2);

    figure;plot(Flex_Control_Spd_Compare_dataset.Time,airspeed_vector_cmd,'b');hold on;plot(Flex_Control_Spd_Compare_dataset.Time,airspeed_vector_meas,'r');xlabel('time (sec)'); ylabel('eas (m/s)');title(['Flex Mode Control Performance at ', num2str(eass(1)), ' m/s and ',num2str(alts(1)),' m']);legend('eas command ','eas ');grid on;
    figure;plot(Flex_Control_Spd_Compare_dataset.Time,Flex_Control_Spd_Compare_dataset.Data(:,4),'r');hold on;plot(Flex_Control_Spd_Compare_dataset.Time,Flex_Control_Spd_Compare_dataset.Data(:,6),'b');xlabel('time (sec)'); ylabel('nz (m/s/s) , Elv_flex_deg');title(['Flex Mode Control Performance at ', num2str(eass(1)), ' m/s and ',num2str(alts(1)),' m']);legend('nz offcenter blended (m/s/s)','Flex control elevator command (deg)');grid on;
    figure;plot(Flex_Control_Spd_Compare_dataset.Time,Flex_Control_Spd_Compare_dataset.Data(:,3),'b');xlabel('time (sec)'); ylabel('alt (m)');title(['Flex Mode Control Performance at ', num2str(eass(1)), ' m/s and ',num2str(alts(1)),' m']);legend('alt ');grid on;

end

%% Get all the line handles from root and set its width
allLineHandles = findall(groot, 'Type', 'line');
set(allLineHandles, 'LineWidth', 2.0);

%% Get all axes handles and set its color
allAxesHandles = findall(groot, 'Type', 'Axes');
set(allAxesHandles, 'FontName','Arial','FontWeight','Bold','LineWidth',3,...
    'FontSize',12);

%% Get titles
alltext = findall(groot, 'Type', 'Text');
set(alltext,'FontName','Arial','FontWeight','Bold','FontSize',12, 'Interpreter', 'None');
