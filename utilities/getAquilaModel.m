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


if (optOptions.Optimize4MultipleFlts ~= TRUE )

    if (optOptions.UseCustomModel ~= TRUE)
        
        ialt            = find(ismember(Aquila.linear.index.alt, alts)) ;
        ieas            = find(ismember(Aquila.linear.index.eas, eass)) ;
        igamma          = find(ismember(Aquila.linear.index.gamma, gammas)) ;
        iphi            = find(ismember(Aquila.linear.index.phi, phis)) ;
        idelta_spoiler  = 1;

        state_0   = Aquila.linear.trim(ialt, ieas, igamma, iphi).x;
        u_0       = Aquila.linear.trim(ialt, ieas, igamma, iphi).u;

        [y_disp, y_vel, y_accel, y_rel, y_control, y_thrust]    = Aquila.nonlinear.compute_output(state_0, u_0);

        trim_idx = {ialt, ieas, igamma, iphi,idelta_spoiler } ;

        disp('.........');
        disp('..........');
        disp('Selected flight condition is:');
        % print the details of the trim point
        Aquila.print_trim_point(trim_idx{:});

        if( optOptions.ShowOpenLoopFltDyn == true )
            Flags_computed    = Aquila.linear.trim(ialt, ieas, igamma, iphi).computed;

            Flags_exist       = Aquila.linear.trim(ialt, ieas, igamma, iphi).exist;
            disp(['Trim existence flag (solution convergence) for this flight condition is ',num2str(Flags_exist)]);


        %           1st element: alt index
        %           2nd element: eas index
        %           3rd element: gamma index
        %           4th element: bank index
        %           5th element: spoiler index


            % create ploter and add the Aircraft objects as sources for eigenvaules
            plotter = EigenvaluePlotter();
            plotter.add_source(Aquila.version, Aquila);
            plotter.nmodes_label = 4;

            % plot a comparison
            % names = {aircraft_1601.version, aircraft_1601_Pre2.version};
            names = {Aquila.version};

            figure(561);
            plotter.show_modes = true;
            plotter.plot_comparison(names, trim_idx);
            xlim([-8., 1.]);
            ylim([-1., 50.]);

            figure(562)
            plotter.freq_high = 2.;
            plotter.plot_comparison(names, trim_idx);
            xlim([-8., 1.]);
            ylim([-1., 2. * 2 * pi()]);

            figure(566);plot3(Aquila.nonlinear.nodes_plot(:,1),Aquila.nonlinear.nodes_plot(:,2),Aquila.nonlinear.nodes_plot(:,3),'r.');
            grid on; box;xlim([-15,15]);ylim([-22,22]);zlim([-10,10]);title('Aquila airplane nodes');xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');


            % Thrust and Torque 
            disp('.........');
            Torque_   = u_0(6:9)' ;
            disp('Trim motor torques (in N.m for each motor) - descending order from the outer left motor  ');
            disp('Positive sign for CW rotation viewing from the rear');
            disp(num2str(Torque_'));

            disp('.........');
            Thrust_   = y_thrust' ;
            disp('Trim motors thrust (in N for each motor)');
            disp(num2str(Thrust_));

            disp('.........');
            disp('.........');
            delta_surface_    = (y_control/Deg2Rad)' ;                      %first element is right control surface and second is left
            disp('control surfaces deflections (in deg) at trim are (descending order from the left elevon):');
            disp(num2str(delta_surface_));

            disp('.........');
            delta_surface_    = (y_control/Deg2Rad)' ;                      %first element is right control surface and second is left
            disp('propeller disk center locations are:');
            disp(Aquila.nonlinear.positions);

            disp('.........');
            %Alpha and Beta trims
            Alpha_trim_                  = y_rel(3)/Deg2Rad ;
            disp('trim angle of attack (@mean axes) is (in deg) :');
            disp(num2str(Alpha_trim_));

            disp('.........');
            Beta_trim_                   = y_rel(2)/Deg2Rad ;
            disp('trim angle of side of slip (@mean axes) is (in deg):');
            disp(num2str(Beta_trim_));


        end

        % First sensor suite for flight control laws (rigid body modes) at IMU:
        torque_const    = 45.2 ;
        throttle_mixing = 0.36 ;
        output_at_node  = 1 ;    %output_at_node: flag to specify mean axis or node; output at mean axis , 1:output at the node specified by node_idx 
        node_idx        = 1 ;    %for now, sensor at the IMU location is chosen for the flight (rigid body) control

        if ( node_idx == 1 )
            node_str = 'IMU sensor';
        elseif ( node_idx == 2 )
            node_str = 'Left inner motor sensor';
        elseif ( node_idx == 3 )
            node_str = 'Left outer motor sensor';
        elseif ( node_idx == 4 )
            node_str = 'Left winglet sensor';
        elseif ( node_idx == 5 )
            node_str = 'Right winglet sensor';
        elseif ( node_idx == 6 )
            node_str = 'Right outer motor sensor';
        elseif ( node_idx == 7 )
            node_str = 'Right inner motor sensor';
        end

        propulsion_parameters = [torque_const, throttle_mixing];
        node_analysis_info    = [output_at_node , node_idx];


        [sys] = ase_lin_sys_gen(Aquila,trim_idx, node_analysis_info ,propulsion_parameters, AquilaModelInputNames, AquilaModelOutputNames) ;
    
    else
        % Use user defined system, such as a systemID model or a ROM from aswing.
        sys = load(['./CustomModels/StateSpace/' optOptions.CustomModelName '.mat']);
        sys = sys.sys;
        assignin('base','ss6',sys);  % Write to linearModel.slx for debugging
        % Reorder system to match control toolbox linear system
        sys = sysTransform(sys,FltCondition.alts(1));
        node_str = 'IMU sensor';
    end
    
    disp('..........');
    disp(['Sensor measurements at the node location ', node_str,' is used for flight (rigid body) control']);
    disp('..........');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% From Model reduction tool %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 1. Model reduction by simplification: minreal ->model reduction by cancelling near by pole/zero pairs, or the "non-minimal" state dynamics - all the uncontrollable
    %     and unobservable modes have been removed.
    %     default Tolerance for zero-pole cancellation is TOL=SQRT(EPS), we reduce it to 1e-10
    %
    
    % Select the second/third sensor suites for flexible modes control -
    % Sensor selection/blending is based on the analysis carried out by using the AquilaFltDynAnalysis.m script  
    if( (optOptions.FlexControlActive == true || optOptions.GLAControlActive == true) && optOptions.UseCustomModel ~= true)
        Analysis_node_table = array2table(Analysis_index_nodes,'RowNames',Nodes_index_rows);

        disp(Analysis_node_table);

        disp('..........');
        node_idx_array_input = input(' Enter a node/sensor index from the above table for flexible modes control - for sensor blending enter the second node index (in an array e.g. [1 2]) :   ');
        
        offcenter_node_idx_1          = node_idx_array_input(1) ;

         if ( offcenter_node_idx_1 == 1 )
            flex_node_str = 'IMU sensor';
        elseif ( offcenter_node_idx_1 == 2 )
            flex_node_str = 'Left inner motor sensor';
        elseif ( offcenter_node_idx_1 == 3 )
            flex_node_str = 'Left outer motor sensor';
        elseif ( offcenter_node_idx_1 == 4 )
            flex_node_str = 'Left winglet sensor';
        elseif ( offcenter_node_idx_1 == 5 )
            flex_node_str = 'Right winglet sensor';
        elseif ( offcenter_node_idx_1 == 6 )
            flex_node_str = 'Right outer motor sensor';
        elseif ( offcenter_node_idx_1 == 7 )
            flex_node_str = 'Right inner motor sensor';
        end
        
        
        if( length(node_idx_array_input) > 1 )
    
            offcenter_node_idx_2 = node_idx_array_input(2) ;
    
            if ( offcenter_node_idx_2 == 1 )
                blending_node_str = 'IMU sensor';
            elseif ( offcenter_node_idx_2 == 2 )
                blending_node_str = 'Left inner motor sensor';
            elseif ( offcenter_node_idx_2 == 3 )
                blending_node_str = 'Left outer motor sensor';
            elseif ( offcenter_node_idx_2 == 4 )
                blending_node_str = 'Left winglet sensor';
            elseif ( offcenter_node_idx_2 == 5 )
                blending_node_str = 'Right winglet sensor';
            elseif ( offcenter_node_idx_2 == 6 )
                blending_node_str = 'Right outer motor sensor';
            elseif ( offcenter_node_idx_2 == 7 )
                blending_node_str = 'Right inner motor sensor';
            end
        end
        
        disp('..........');
        disp('..........');
        disp([' sensor measurements at the node location ', flex_node_str,' is used for flexible mode control']);
        disp('..........');
        
        node_analysis_info_flex_1      = [output_at_node , offcenter_node_idx_1];
        [sys_offcenter_node_1]         = ase_lin_sys_gen(Aquila,trim_idx, node_analysis_info_flex_1 ,propulsion_parameters, AquilaModelInputNames, AquilaModelOutputNames) ;    

        if( length(node_idx_array_input) > 1 )
            
            disp([' sensor measurements at the node location ', blending_node_str,' is blended into the previous node sensor for flexible mode control']);
            node_analysis_info_flex_2      = [output_at_node , offcenter_node_idx_2];
            [sys_offcenter_node_2]         = ase_lin_sys_gen(Aquila,trim_idx, node_analysis_info_flex_2 ,propulsion_parameters, AquilaModelInputNames, AquilaModelOutputNames) ;    
            % construct the second system for blending from the selected input to the selected output
            % sys_full_blending_node         = sys_blending_node(blending_output_idx,input_idx) ;
            % figure;step(sys_full_blending_node*Deg2Rad,[0:0.05:5]);grid;title(['Step response from ',char(Control_inputs(input_idx)),' input to ', char(Output_signals(blending_output_idx)), ' output measured at node location ', blending_node_str]);

        end

        % Construct the aircraft output model at the off-center node
        if( length(node_idx_array_input) > 1 )
              
            % Get the blended sensor mimo system for flexible modes control
            sys_flex_mimo_full     = parallel(sys_offcenter_node_1,sys_offcenter_node_2);
            
            % Get the blended sensor siso system for flexible modes control from optOptions.FlexControlInputIndex to optOptions.FlexControlOutputIndex --> Used during the design process
            sys_flex_siso_full     = parallel(sys_offcenter_node_1(optOptions.FlexControlOutputIndex,optOptions.FlexControlInputIndex),sys_offcenter_node_2(optOptions.FlexControlOutputIndex,optOptions.FlexControlInputIndex));
            
            % Note that for different sensor blending combinations, the minreal_Tol must be adjusted.
            minreal_Tol = 1e-10; % Set tolerance (increasing it, will force additional cancellations)
            sys_flex_minreal = minreal(sys_flex_siso_full,minreal_Tol, false);   %sys_blend_full
            disp('..........');
            figure;rlocusplot((-0.5)*sys_flex_minreal);grid; xlim([-10,5]);ylim([0,45]);title(['Root locus from blended sensor outputs at ', flex_node_str, ' and ', blending_node_str ]); 

        else
            % Get the mimo system for flexible modes control
            sys_flex_mimo_full     = sys_offcenter_node_1 ;

            % Get the siso system for flexible modes control from optOptions.FlexControlInputIndex to optOptions.FlexControlOutputIndex
            sys_flex_siso_full     = sys_offcenter_node_1(optOptions.FlexControlOutputIndex,optOptions.FlexControlInputIndex) ;
            
            minreal_Tol = 1e-10;
            sys_flex_minreal = minreal(sys_flex_siso_full,minreal_Tol, false);
            disp('..........');
            figure;rlocusplot((-1)*sys_flex_minreal);grid; xlim([-10,5]);ylim([0,45]);title(['Root locus from sensor output at ', flex_node_str ]); 
            figure;step(sys_flex_siso_full*Deg2Rad,[0:0.05:20],'r');grid;hold on;step(sys_flex_minreal(1,1)*Deg2Rad,[0:0.05:20],'b.');

        end
        
        
    end
    
    
    % sys_scaled = prescale(sys,{10^-3,10^2}) ;
    % sys_minreal = minreal(sys_scaled,minreal_Tol, false);
    
    minreal_Tol = 1e-10; % Set tolerance (increasing it, will force additional cancellations)
    sys_minreal = minreal(sys,minreal_Tol, false);
        
  
    %% 2.a Model reduction by truncation and residulization: Balreal+Modred == Balred
    % OPT = stabsepOptions('Offset', 1e-6 ,'RelTol', 1e-8);
    hsvd_offset = 0.0001 ; %offset to determine unstable poles
    opts = hsvdOptions('Offset',hsvd_offset); 
    [sys_balreal,g] = balreal(sys,opts) ;                               % compute balanced realization
    % g_noInf = g(~isinf(g)) ;                                          % remove the entries corrsponding to the unstable mdoes
    % g_max           = max(g_noInf) ;
    % g_normalized    = g./g_max ;


    elim = (g<1e-1) ;                                                   % identify states with smallest grammian g -> negligible states
    % elim = (g_normalized<1e-4) ;                                      % identify states with relative small grammians (1% of the largest grammian g -> negligible states)
    sys_modred_residual  =  modred(sys_balreal,elim,'matchdc');         % remove negligible states by using residulization, "matchdc" -> default

    sys_modred_trunct    =  modred(sys_balreal,elim,'truncate');        % remove negligible states by using truncation

    %% 3 Model reduction by similarity transformation

    %%% reduce model size
    fmax=3;  %reduced model Hz 
    sys_modal=canon(sys,'modal', 1e6);  % Similarity transformation and canonical form
    A_modal = sys_modal.a;
    freqs   = sqrt(diag(A_modal).*diag(A_modal) + vertcat(diag(A_modal,1),  0.).*vertcat(diag(A_modal,1),  0.) + vertcat( 0., diag(A_modal,-1)).*vertcat( 0., diag(A_modal,-1)));
    elim    = (freqs>fmax*2*pi);
    sys_modred_modalred = modred(sys_modal,elim);

    % Select either full order model or one of the reduced order models should be used during the control laws design process
    if (optOptions.UseFullOrderModel == true )
        % full order model state space matrices at the IMU location (node 1)
        disp('........');
        disp(['System (full order) with outputs measured at the node location ', node_str,' is used for control laws feedback/design']);
        disp('..........');
        LIN_AC_S_A = sys.A ;
        LIN_AC_S_B = sys.B ;
        LIN_AC_S_C = sys.C ;
        LIN_AC_S_D = sys.D ;

        if( (optOptions.FlexControlActive == true || optOptions.GLAControlActive == true ) && optOptions.UseCustomModel ~= true)
            % Do not use the minimal realization model for Simulink simulation - Only full order model must be used. The minimal
            % realization model can be used for the Flexible mode control system design though
            % LIN_FLEX_AC_S_A = sys_flex_minreal.A ;
            % LIN_FLEX_AC_S_B = sys_flex_minreal.B ;
            % LIN_FLEX_AC_S_C = sys_flex_minreal.C ;
            % LIN_FLEX_AC_S_D = sys_flex_minreal.D ;
            
            %Only for the siso analysis on the closed loop flexible modes control
            % LIN_FLEX_AC_S_A = sys_flex_siso_full.A ;
            % LIN_FLEX_AC_S_B = sys_flex_siso_full.B ;
            % LIN_FLEX_AC_S_C = sys_flex_siso_full.C ;
            % LIN_FLEX_AC_S_D = sys_flex_siso_full.D ;
            
            
            LIN_FLEX_AC_S_A = sys_flex_mimo_full.A ;
            LIN_FLEX_AC_S_B = sys_flex_mimo_full.B ;
            LIN_FLEX_AC_S_C = sys_flex_mimo_full.C ;
            LIN_FLEX_AC_S_D = sys_flex_mimo_full.D ;


        end

    elseif (optOptions.UseResidulizedReducedOrder == true )
        % reduced order model state space matrices at the IMU location
        % (node 1) - Reduced by residulization
        disp('........');
        disp(['System (lag residualized) with outputs measured at the node location ', node_str,' is used for control laws feedback/design']);
        disp('..........');

        LIN_AC_S_A = sys_modred_residual.A ;
        LIN_AC_S_B = sys_modred_residual.B ;
        LIN_AC_S_C = sys_modred_residual.C ;
        LIN_AC_S_D = sys_modred_residual.D ;
    elseif ( optOptions.UseTruncatedReducedOrder == true )
        % reduced order model state space matrices at the IMU location
        % (node 1) - Reduced by truncation
        disp('........');
        disp(['System (Truncated) with outputs measured at the node location ', node_str,' is used for control laws feedback/design']);
        disp('..........');
        LIN_AC_S_A = sys_modred_trunct.A ;
        LIN_AC_S_B = sys_modred_trunct.B ;
        LIN_AC_S_C = sys_modred_trunct.C ;
        LIN_AC_S_D = sys_modred_trunct.D ;
    else
        error(' Either full order model or one of the reduced order models must be chosen in OptConfigSetup.m script');
    end

    if( optOptions.PlotOpenLoopFltDyn == true )
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% From Model reduction tool %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 1. Model reduction by simplification: minreal ->model reduction by cancelling near by pole/zero pairs, or the "non-minimal" state dynamics - all the uncontrollable
        %               and unobservable modes have been removed.
        % The Pole and zeros of the other input and output pairs can be also
        % included
        fprintf('...............\n');
        disp(' Poles of the q_radps/elvCmd_rad transfer function for the full-order model');

        % The Pole and zeros of the other input and output pairs can be also
        % included
        damp(sys('q_radps','elvCmd_rad'));


        disp('.........');
        disp('"pole-zero cancellation" reduced order model size:');
        disp(size(sys_minreal.A));

        fprintf('...............\n');
        disp(' Poles of the reduced order (pole/zero cancellation) q_radps/elvCmd_rad transfer function ');

        damp(sys_minreal('q_radps','elvCmd_rad'));


        %% 2.a Model reduction by truncation and residulization: Balreal+Modred == Balred
        disp('"residulization" reduced order model size:');
        disp(size(sys_modred_residual.A));

        disp('"Truncation" reduced order model size:');
        disp(size(sys_modred_trunct.A));

        %% 3 Model reduction by similarity transformation

        %%% reduce model size
        disp('"Modal reduction" reduced order model size:');
        disp(size(sys_modred_modalred.A));
        
        % 
        figure(580); pzmap(sys('q_radps','elvCmd_rad'),'r',sys_balreal('q_radps','elvCmd_rad'),'b');sgrid;legend('full order model','full order model - Balanced');title('q_radps/elvCmd_rad transfer function - pole/zeros', 'Interpreter', 'none');

        
        %Compare the bode plots:
        P = bodeoptions ; P.PhaseWrapping = 'on' ;P.grid ='on'; P.TickLabel.FontSize = 12; P.Title.FontSize = 10;

        %Collective elevon to Pitch rate
        figure(601);bode(sys('q_radps','elvCmd_rad'),'r',P);
        figure(601);hold on;bode(sys_minreal('q_radps','elvCmd_rad'),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
        % set(0,'DefaultAxesFontSize',18); set(0,'DefaultLineLinewidth',2)
        title('q_radps/elvCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','pole-zero cancellation');

        figure(605); pzmap(sys('q_radps','elvCmd_rad'),'r',sys_minreal('q_radps','elvCmd_rad'),'b');sgrid;legend('full order model','pole-zero cancellation');title('q_radps/elvCmd_rad transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(602);bode(sys('q_radps','elvCmd_rad'),'r',P);
        figure(602);hold on;bode(sys_modred_residual('q_radps','elvCmd_rad'),'k',P);
        title('Collective elevon to Pitch rate - Frequency response', 'Interpreter', 'none');
        legend('full order model','residulization');

        figure(606); pzmap(sys('q_radps','elvCmd_rad'),'r',sys_modred_residual('q_radps','elvCmd_rad'),'k');sgrid;legend('full order model','residulization');title('q_radps/elvCmd_rad transfer function - Pole/zeros');

        figure(603);bode(sys('q_radps','elvCmd_rad'),'r',P);
        figure(603);hold on;bode(sys_modred_trunct('q_radps','elvCmd_rad'),'m',P);
        % xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
        title('q_radps/elvCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','truncation');

        % see how the pole-zeros might be different in the original model and its
        % approximations
        figure(610); pzmap(sys('q_radps','elvCmd_rad'),'r',sys_modred_trunct('q_radps','elvCmd_rad'),'m');sgrid;legend('full order model','truncation');title('q_radps/elvCmd_rad transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(614);bode(sys('q_radps','elvCmd_rad'),'r',P);
        figure(614);hold on;bode(sys_modred_modalred('q_radps','elvCmd_rad'),'g',P);
        title('q_radps/elvCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','modal selection');

        figure(616); pzmap(sys('q_radps','elvCmd_rad'),'r',sys_modred_modalred('q_radps','elvCmd_rad'),'g');sgrid;legend('full order model','Modal selection');title('q_radps/elvCmd_rad transfer function - Frequency response');
        figure(617); pzmap(sys('q_radps','elvCmd_rad'),'r');sgrid;legend('full order model of Collective elevon to Pitch rate ');
        figure(618); pzmap(sys_modred_modalred('q_radps','elvCmd_rad'),'g');sgrid;legend('Modal selection');title('q_radps/elvCmd_rad transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(620);
        bode(sys('q_radps','elvCmd_rad'),'r',sys_minreal('q_radps','elvCmd_rad'),'b',sys_modred_residual('q_radps','elvCmd_rad'),'k',sys_modred_trunct('q_radps','elvCmd_rad'),'m',sys_modred_modalred('q_radps','elvCmd_rad'),'g',P);
        legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
        title('q_radps/elvCmd_rad transfer function - Frequency response', 'Interpreter', 'none');

        figure(621);
        stepplot(sys('q_radps','elvCmd_rad'),'r',sys_minreal('q_radps','elvCmd_rad'),'b',sys_modred_residual('q_radps','elvCmd_rad'),'k',sys_modred_trunct('q_radps','elvCmd_rad'),'m',sys_modred_modalred('q_radps','elvCmd_rad'),'g',100);
        legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
        title('q_radps/elvCmd_rad transfer function - Time response', 'Interpreter', 'none');

        figure(622);
        stepplot(sys('q_radps','elvCmd_rad'),'r--',sys_modred_residual('q_radps','elvCmd_rad'),'k',sys_modred_trunct('q_radps','elvCmd_rad'),'m',100);
        legend('full order model','residulization','truncation');
        title('q_radps/elvCmd_rad transfer function - time response', 'Interpreter', 'none');


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Differential elevon to roll rate
        figure(631);bode(sys('p_radps','ailCmd_rad'),'r',P);
        figure(631);hold on;bode(sys_minreal('p_radps','ailCmd_rad'),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
        title('p_radps/ailCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','pole-zero cancellation');

        figure(632); pzmap(sys('p_radps','ailCmd_rad'),'r',sys_minreal('p_radps','ailCmd_rad'),'b');sgrid;legend('full order model','pole-zero cancellation');title('p_radps/ailCmd_rad transfer function');

        figure(634);bode(sys('p_radps','ailCmd_rad'),'r',P);
        figure(634);hold on;bode(sys_modred_residual('p_radps','ailCmd_rad'),'k',P);
        title('p_radps/ailCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','residulization');

        figure(636); pzmap(sys('p_radps','ailCmd_rad'),'r',sys_modred_residual('p_radps','ailCmd_rad'),'k');sgrid;legend('full order model','residulization');title('p_radps/ailCmd_rad transfer function - Pole/zeros');

        figure(638);bode(sys('p_radps','ailCmd_rad'),'r',P);
        figure(638);hold on;bode(sys_modred_trunct('p_radps','ailCmd_rad'),'m',P);
        % xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
        title('p_radps/ailCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','truncation');

        % see how the pole-zeros might be different in the original model and its
        % approximations
        figure(640); pzmap(sys('p_radps','ailCmd_rad'),'r',sys_modred_trunct('p_radps','ailCmd_rad'),'m');sgrid;legend('full order model','truncation');title('p_radps/ailCmd_rad transfer function - Pole/zeros');

        figure(642);bode(sys('p_radps','ailCmd_rad'),'r',P);
        figure(642);hold on;bode(sys_modred_modalred('p_radps','ailCmd_rad'),'g',P);
        title('p_radps/ailCmd_rad transfer function - Frequency response', 'Interpreter', 'none');
        legend('full order model','modal selection');

        figure(644); pzmap(sys('p_radps','ailCmd_rad'),'r',sys_modred_modalred('p_radps','ailCmd_rad'),'g');sgrid;legend('full order model','Modal selection');title('p_radps/ailCmd_rad transfer function - Pole/zeros');

        figure(646);
        bode(sys(8,2),'r',sys_minreal('p_radps','ailCmd_rad'),'b',sys_modred_residual('p_radps','ailCmd_rad'),'k',sys_modred_trunct('p_radps','ailCmd_rad'),'m',sys_modred_modalred('p_radps','ailCmd_rad'),'g',P);
        legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
        title('p_radps/ailCmd_rad transfer function - Frequency response', 'Interpreter', 'none');

        figure(648);
        stepplot(sys('p_radps','ailCmd_rad'),'r--',sys_modred_residual('p_radps','ailCmd_rad'),'k',sys_modred_trunct('p_radps','ailCmd_rad'),'m',30);
        legend('full order model','residulization','truncation');
        title('p_radps/ailCmd_rad transfer function - time response', 'Interpreter', 'none');


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Differential throttle to roll rate
        figure(651);bode(sys('p_radps','diffThrCmd_Norm'),'r',P);
        figure(651);hold on;bode(sys_minreal('p_radps','diffThrCmd_Norm'),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
        title('p_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');
        legend('full order model','pole-zero cancellation');

        figure(652); pzmap(sys('p_radps','diffThrCmd_Norm'),'r',sys_minreal('p_radps','diffThrCmd_Norm'),'b');sgrid;legend('full order model','pole-zero cancellation');title('p_radps/diffThrCmd_Norm transfer function - Pole/zeros');

        figure(654);bode(sys('p_radps','diffThrCmd_Norm'),'r',P);
        figure(654);hold on;bode(sys_modred_residual('p_radps','diffThrCmd_Norm'),'k',P);
        title('p_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');
        legend('full order model','residulization');

        figure(656); pzmap(sys('p_radps','diffThrCmd_Norm'),'r',sys_modred_residual('p_radps','diffThrCmd_Norm'),'k');sgrid;legend('full order model','residulization');title('p_radps/diffThrCmd_Norm transfer function - Pole/zeros');

        figure(658);bode(sys('p_radps','diffThrCmd_Norm'),'r',P);
        figure(658);hold on;bode(sys_modred_trunct('p_radps','diffThrCmd_Norm'),'m',P);
        % xlabel('Frequency (rad)','FontSize',12,'FontWeight','bold','Color','r')
        title('p_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');
        legend('full order model','truncation');

        % see how the pole-zeros might be different in the original model and its
        % approximations
        figure(660); pzmap(sys('p_radps','diffThrCmd_Norm'),'r',sys_modred_trunct('p_radps','diffThrCmd_Norm'),'m');sgrid;legend('full order model','truncation');title('p_radps/diffThrCmd_Norm transfer function - Pole/zeros');

        figure(662);bode(sys('p_radps','diffThrCmd_Norm'),'r',P);
        figure(662);hold on;bode(sys_modred_modalred('p_radps','diffThrCmd_Norm'),'g',P);
        title('p_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');
        legend('full order model','modal selection');

        figure(664); pzmap(sys('p_radps','diffThrCmd_Norm'),'r',sys_modred_modalred('p_radps','diffThrCmd_Norm'),'g');sgrid;legend('full order model','Modal selection');

        figure(666);
        bode(sys('p_radps','diffThrCmd_Norm'),'r',sys_minreal('p_radps','diffThrCmd_Norm'),'b',sys_modred_residual('p_radps','diffThrCmd_Norm'),'k',sys_modred_trunct('p_radps','diffThrCmd_Norm'),'m',sys_modred_modalred('p_radps','diffThrCmd_Norm'),'g',P);
        legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
        title('p_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');

        figure(668);
        stepplot(sys('p_radps','diffThrCmd_Norm'),'r--',sys_modred_residual('p_radps','diffThrCmd_Norm'),'k',sys_modred_trunct('p_radps','diffThrCmd_Norm'),'m',50);
        legend('full order model','residulization','truncation');
        title('p_radps/diffThrCmd_Norm - time response', 'Interpreter', 'none');

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Differential throttle to yaw rate
        figure(671);bode(sys('r_radps','diffThrCmd_Norm'),'r',P);
        figure(671);hold on;bode(sys_minreal('r_radps','diffThrCmd_Norm'),'b',P);    %note that sys_minreal doesn't include the uncontrllable and unobservable modes of the original sys
        title('r_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');
        legend('full order model','pole-zero cancellation');

        figure(672); pzmap(sys('r_radps','diffThrCmd_Norm'),'r',sys_minreal('r_radps','diffThrCmd_Norm'),'b');sgrid;legend('full order model','pole-zero cancellation');title('r_radps/diffThrCmd_Norm transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(674);bode(sys('r_radps','diffThrCmd_Norm'),'r',P);
        figure(674);hold on;bode(sys_modred_residual('r_radps','diffThrCmd_Norm'),'k',P);
        title('r_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');
        legend('full order model','residulization');

        figure(676); pzmap(sys('r_radps','diffThrCmd_Norm'),'r',sys_modred_residual('r_radps','diffThrCmd_Norm'),'k');sgrid;legend('full order model','residulization');title('r_radps/diffThrCmd_Norm transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(678);bode(sys('r_radps','diffThrCmd_Norm'),'r',P);
        figure(678);hold on;bode(sys_modred_trunct('r_radps','diffThrCmd_Norm'),'m',P);
        title('r_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');   
        legend('full order model','truncation');

        % see how the pole-zeros might be different in the original model and its
        % approximations
        figure(680); pzmap(sys('r_radps','diffThrCmd_Norm'),'r',sys_modred_trunct('r_radps','diffThrCmd_Norm'),'m');sgrid;legend('full order model','truncation');title('r_radps/diffThrCmd_Norm transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(682);bode(sys('r_radps','diffThrCmd_Norm'),'r',P);
        figure(682);hold on;bode(sys_modred_modalred('r_radps','diffThrCmd_Norm'),'g',P);
        title('r_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');    
        legend('full order model','modal selection');

        figure(684); pzmap(sys('r_radps','diffThrCmd_Norm'),'r',sys_modred_modalred('r_radps','diffThrCmd_Norm'),'g');sgrid;legend('full order model','Modal selection');title('r_radps/diffThrCmd_Norm transfer function - Pole/zeros', 'Interpreter', 'none');

        figure(686);
        bode(sys('q_radps','diffThrCmd_Norm'),'r',sys_minreal('r_radps','diffThrCmd_Norm'),'b',sys_modred_residual('r_radps','diffThrCmd_Norm'),'k',sys_modred_trunct('r_radps','diffThrCmd_Norm'),'m',sys_modred_modalred('r_radps','diffThrCmd_Norm'),'g',P);
        legend('full order model','pole-zero cancellation','residulization','truncation','modal selection');
        title('r_radps/diffThrCmd_Norm - Frequency response', 'Interpreter', 'none');

        figure(688);
        stepplot(sys('r_radps','diffThrCmd_Norm'),'r--',sys_modred_residual('r_radps','diffThrCmd_Norm'),'k',sys_modred_trunct('r_radps','diffThrCmd_Norm'),'m',20);
        legend('full order model','residulization','truncation');
        title('r_radps/diffThrCmd_Norm - Time response', 'Interpreter', 'none');


        figure(690);bode(sys('pres_alt_m','thrCmd_Norm'));grid;title('pres_alt_m/thrCmd_Norm transfer function', 'Interpreter', 'none');
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Write the model and flag data into the base workspace
    assignin('base','Ts',Ts);

    assignin('base','LIN_AC_S_A',LIN_AC_S_A);
    assignin('base','LIN_AC_S_B',LIN_AC_S_B);
    assignin('base','LIN_AC_S_C',LIN_AC_S_C);
    assignin('base','LIN_AC_S_D',LIN_AC_S_D);
    
    assignin('base','FlexControlActive',optOptions.FlexControlActive);
    
    if( optOptions.FlexControlActive == true && optOptions.UseCustomModel == true )
        % Load twist system
        sys = load(['./CustomModels/StateSpace/' optOptions.CustomModelName '_twist.mat']);
        sys = sys.sys_twist;
        % Compute twist from the three theta values
        sys = sysTransformTwist(sys);
        
        assignin('base','LIN_FLEX_AC_S_A',sys.A);
        assignin('base','LIN_FLEX_AC_S_B',sys.B);
        assignin('base','LIN_FLEX_AC_S_C',sys.C);
        assignin('base','LIN_FLEX_AC_S_D',sys.D);
        
    elseif( optOptions.FlexControlActive == true || optOptions.GLAControlActive == true )
        assignin('base','LIN_FLEX_AC_S_A',LIN_FLEX_AC_S_A);
        assignin('base','LIN_FLEX_AC_S_B',LIN_FLEX_AC_S_B);
        assignin('base','LIN_FLEX_AC_S_C',LIN_FLEX_AC_S_C);
        assignin('base','LIN_FLEX_AC_S_D',LIN_FLEX_AC_S_D);
    end
    assignin('base','GLAControlActive',optOptions.GLAControlActive);
else
    DynPres_Pa    = zeros(size(alts,2),size(eass,2)) ; 
    pres_alt_m    = zeros(size(alts,2),size(eass,2)) ;
    
    igamma          = find(ismember(Aquila.aircraft.linear.index.gamma, gammas(1))) ;
    iphi            = find(ismember(Aquila.aircraft.linear.index.phi, phis(1))) ;
    idelta_spoiler  = 1;

    
    for ii=1:size(alts,2)
        
        ialt            = find(ismember(Aquila.aircraft.linear.index.alt, alts(ii))) ;

        for jj=1:size(eass,2)
            
            ieas            = find(ismember(Aquila.aircraft.linear.index.eas, eass(jj))) ;

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

            %get the air density in kg/m^3 at the corresponding height
            [T_0, a_0, P_0, rho_0] = atmosisa(state_0(3)) ;

            DynPresPa = round((0.5*rho_0*(y_vel(1))^2)) ;
            DynPres_Pa(ii,jj)  = DynPresPa ;
            pres_alt_m(ii,jj)  = state_0(3) ;


            % Get the linearzied input and feedforward matrices at the IMU location 
            
            %  1st element: alt index
            %  2nd element: eas index
            %  3rd element: gamma index
            %  4th element: bank index
            %  5th element: spoiler index

            trim_idx = [ialt, ieas, igamma, iphi,idelta_spoiler ] ;
            torque_const    = 45.2 ;
            throttle_mixing = 0.36 ;
            output_at_node  = 1 ;       %output_at_node: flag to specify mean axis or node
                                        %0: output at mean axis , 1:output at the node specified by node_idx
            
            Aquila.aircraft.linear.finite_difference=1e-5; % perturbation used for linearization
                         
            [sys] = ase_lin_sys_gen(Aquila,trim_idx, output_at_node , 1 , 0 ,torque_const,throttle_mixing) ; 
            

            % find the model state space matrices at the IMU location (node 1)
            LIN_AC_S_A = sys.A ;
            LIN_AC_S_B = sys.B ;
            LIN_AC_S_C = sys.C ;
            LIN_AC_S_D = sys.D ;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % I put the data here until we can manage the memory better
            assignin('base','Ts',optOptions.Ts);

            assignin('base','LIN_AC_S_A',LIN_AC_S_A);
            assignin('base','LIN_AC_S_B',LIN_AC_S_B);
            assignin('base','LIN_AC_S_C',LIN_AC_S_C);
            assignin('base','LIN_AC_S_D',LIN_AC_S_D);

            Aquila_LIN(:,:,ii,jj) = ss(LIN_AC_S_A,LIN_AC_S_B,LIN_AC_S_C,LIN_AC_S_D) ;
        end
    end
    
    %Introduce labels to the columns and rows of the state space model
    Aquila_LIN.u = {'elvCmd_rad' 'ailCmd_rad' 'thrCmd_Norm' 'diffThrCmd_Norm' 'splCmd_rad'};
    Aquila_LIN.y = {'' '' 'pres_alt_m' 'tas_mps' 'phi_rad' 'theta_rad' 'psi_rad' 'p_radps' 'q_radps' 'r_radps' '' '' '' 'a_x_IMU_mpss' '' 'a_z_IMU_mpss' '' 'DynPres_Pa' 'OAT' 'eas_mps' 'aoa_rad' 'aos_rad' '' '' '' 'gama_rad'};

    assignin('base','Aquila_LIN',Aquila_LIN);
    
    if( optOptions.PlotOpenLoopFltDyn == true )
        figure(2002);hold on;sigma(Aquila_LIN(Aquila_LIN.y(9),Aquila_LIN.u(1),:,:),{10^-3,10^2},'r');legend('q/d_elv - open loop response');

        figure(2003);hold on;sigma(Aquila_LIN(Aquila_LIN.y(3),Aquila_LIN.u(3),:,:),{10^-3,10^2},'r');legend('h/d_thr - open loop response');

        figure(2004);hold on;sigma(Aquila_LIN(Aquila_LIN.y(10),Aquila_LIN.u(4),:,:)*(180/pi),{10^-3,10^2},'r');legend('r(deg)/d_Diffthr - open loop response');

        figure(2005);hold on;sigma(Aquila_LIN(Aquila_LIN.y(8),Aquila_LIN.u(4),:,:)*(180/pi),{10^-3,10^2},'r');legend('p(deg)/d_Diffthr - open loop response');

        figure(2006);hold on;sigma(Aquila_LIN(Aquila_LIN.y(8),Aquila_LIN.u(2),:,:),{10^-3,10^2},'r');legend('p/d_Diffelv - open loop response');

        figure(2007);hold on;sigma(Aquila_LIN(Aquila_LIN.y(5),Aquila_LIN.u(2),:,:),{10^-3,10^2},'r');legend('phi/d_Diffelv - open loop response');

        figure(2008);hold on;sigma(Aquila_LIN(Aquila_LIN.y(22),Aquila_LIN.u(2),:,:),{10^-3,10^2},'r');legend('AoS/d_Diffelv - open loop response');

        figure(2009);hold on;sigma(Aquila_LIN(Aquila_LIN.y(10),Aquila_LIN.u(2),:,:),{10^-3,10^2},'r');legend('r/d_Diffelv - open loop response');

        figure(2010);hold on;sigma(Aquila_LIN(Aquila_LIN.y(5),Aquila_LIN.u(4),:,:),{10^-3,10^2});legend('phi/d_Diffthr - open loop response');
        figure(2010);hold on;sigma(Aquila_LIN(Aquila_LIN.y(10),Aquila_LIN.u(4),:,:),{10^-3,10^2});legend('phi/d_Diffthr - open loop response','r/d_Diffthr - open loop response');
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% Set the right figure properties:
if( optOptions.PlotOpenLoopFltDyn == true )

    %% Get all the line handles from root and set its width
    allLineHandles = findall(groot, 'Type', 'line');
    set(allLineHandles, 'LineWidth', 2.0);

    %% Get all axes handles and set its color
    allAxesHandles = findall(groot, 'Type', 'Axes');
    set(allAxesHandles, 'FontName','Arial','FontWeight','Bold','LineWidth',2,...
        'FontSize',14);

    %% Get titles
    alltext = findall(groot, 'Type', 'Text');
    set(alltext,'FontName','Arial','FontWeight','Bold','FontSize',14, 'Interpreter', 'None');

end
