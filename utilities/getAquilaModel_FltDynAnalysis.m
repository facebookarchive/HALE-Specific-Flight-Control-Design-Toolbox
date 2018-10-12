%	PURPOSE:            1. Analysis of open loop flight dynamics of High Altitude Long Endurance (HALE) very flexible airplanes 
%                       
%
%	INPUTS:             1. Path to be added to the existing MATLAB paths (following the standard data folder address structure)
%                       2. aircraft build number (v1.3.2) and configuration
%                       version ('Aquila_1601')
%                       3. flight conditions to be analyzed.
%                       4. for a general airplane model structure other than Aquila, an option to only provide the airplane
%                       linearzied state space matrices at different flight conditions
%                       
%                     
%                       
%
% 
% 
% 	OUTPUTS:            1. The aircraft trim condition, location of the sensors and servos including motors, and other geomteric configuration parameters required for flight control laws design
%                       2. Aquila_OL_Model.mat data containing the aircraft state space matrices expressed at the determined sensor location at a given flight condition 
%                       3. Plots the flutter and rigid body modes instability boundaries in the flight envelop 
%                       4. Plots the dry flexible mode shapes 
%                       5. plots the root loci for the sensors located on the wing
%                       6. Plots and saves (in the current matlab directory) the airplane mode variations when sweeping altitude, airspeed, bank attitude and flight path angle 
%                       7. flutter frequency range
%                       8. Degree of Controllability/Observability and the impulse residue metric from inputs to the output modes measured at various nodes
%
%
%
%	PRE-REQUISITES:		ASE model should be available,  					
%
%	LIMITATIONS:		Currently many
%
%
%	DEVELOPER :			
%
%
%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% close all;
clear all;
clc;
% define the constants in the section
% define the analysis flags and switchs here
% include any preliminary information regarding the airplane model folder address, airplane configuration, trim flight condition(s), etc here:

%% constants and parameter definition:
Deg2Rad = pi/180 ;
g_acc   = 9.8 ;                               % (m/s/s) gravity acceleration - consider implementing its variations with altitude 

%%Dashboard (future GUI) - Set the analysis flags and switches:
printAirplaneConfigurationInfo = false ;       % set to true: to print out the Aquila airplane configuration and geometric information 
plotModes                      = false ;       % set to true: to identify contributing modes to individual eigenvalues and print them in a s-plane plot
plotUnstableModes              = false ;       % set to true: to plot the unstable modes across the flight envelope (see also printFlutterOnlyBoundry)  
printFlutterOnlyBoundry        = true ;        % set to true (only active if printUnstableModes flag set to true) : to plot flutter only instability boundry , false: to include rigid body modes instability boundries as well - 
plotSpdAltLoci                 = false ;       % set to true: to plot the locus of the modes with speed, altitude, bank attitude and flight path angle variations
saveFltCnds                    = false ;       % set to true: to save the flight condition parameters in the workspace for the next analyses.
perfomCtrbObsvAnalysis         = false ;       % set to true: to perform "Modal" controllability/observability and Impulse residue analysis for selected modes 
plotTransferFunctions          = false ;        % set to true: to plot the open loop transfer functions for the selected input-output pairs
useSISOtool                    = false ;       % set to true: to deploy the siso design tool


%% path and addresses:
user_name                   = char(java.lang.System.getProperty('user.name'));
ACModelDataPath             = fullfile('/','Users',user_name,'Dropbox (Facebook)','Aquila','GNC','Modeling and Simulation','ASE','Aquila Matlab Simulator v1.3.2 10-July-2017') ;
ACModelBuild                = 'v1.3.2';                      % It is usually defined within the model folder name (see the model folder name defined in the ACModelDataPath field)
ACModelVersion              = 'Aquila_1601';                 % Choose between the existing Aquila model versions including Aquila_1601, Aquila_2, Aquila1A_1

% Determine the trim condition(s) from the available list at which the linearized model will be obtained 
alts   = [ 18000 ] ;      %Altitude from [0, 1500, 3000, 4500, 6000, 7500, 9000, 10500, 12000, 13500, 15000, 16500, 18000, 19500, 21000, 22500, 24000]
eass   = [ 12.44 ] ;      %EAS from [9.1300, 9.9300, 10.9700, 12.4400, 14.7200}
gammas = [ 0 ] ;          %Flight path angle from [ -5, -4, -3, -1,  0,  1,  2,  3,  4,  6,  8 ]
phis   = [ 0 ] ;          %Bank attitude from [0, 2, 5, 10]
delsps = [ 0 ] ;          %delta spoilers from [0, 20, 40, 60, 70]

%Aircraft model inputs and outputs based on the settings in the
%ase_lin_sys_gen.m script and the Simulink linear model block
AquilaModelInputNames       = {'elvCmd_rad' 'ailCmd_rad' 'thrCmd_Norm' 'diffThrCmd_Norm' 'splCmd_rad'};   %Names of the Aquila model input commands in the Simulink model 
AquilaModelOutputNames      = {'p_n_m' 'p_e_m' 'pres_alt_m' 'tas_mps' 'phi_rad' 'theta_rad' 'psi_rad' 'p_radps' 'q_radps' 'r_radps' 'v_n_mps' 'v_e_mps' 'v_d_mps' 'a_x_IMU_mpss' 'a_y_IMU_mpss' 'a_z_IMU_mpss' 'StatPres_Pa' 'DynPres_Pa' 'OAT' 'eas_mps' 'aoa_rad' 'aos_rad' 'v_xb_mps' 'v_yb_mps' 'v_zb_mps' 'gama_rad'}; %Names of the Aquila model output signals in the Simulink model

% Settings to generate the state space (A,B,C, D) matrices with considering sensors either on the airplane mean axes or at 
% either of the nodes on the aircraft body (determined by node_idx)

%         output_at_node:       flag to specify mean axis or node
%                               0: output at mean axis , 1:output at the node specified by node_idx
%         
%         node_idx:             1: IMU node
%                               2: Left inner motor node
%                               3: Left outer motor node
%                               4: Left wingtip
%                               5: Right wingtip
%                               6: Right outer motor node
%                               7: Right inner motor node

%  Should be included in an interactive GUI
output_at_node  = 1 ;    

%Provide list of the aircraft analysis nodes here with the associated indeces:
Analysis_index_nodes   = {'IMU node';'Left inner motor node';'Left outer motor node';'Left wingtip';'Right wingtip';'Right outer motor node';'Right inner motor node'};
Nodes_index_rows       = {'1'       ; '2'                   ;'3'                    ;'4'           ;'5'            ;'6'                     ;'7'};          

Analysis_node_table = array2table(Analysis_index_nodes,'RowNames',Nodes_index_rows);

disp(Analysis_node_table);

disp('..........');
node_idx = input(' Enter a node index from the above table ...');

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

% Aircraft model constants and parameters 
torque_const    = 45.2 ;
throttle_mixing = 0.36 ;


%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

sim_folder = fullfile(ACModelDataPath,'model','aircraft','ase','aquila_matlab_simulator') ;
addpath(genpath(sim_folder));

model_directory = fullfile(ACModelDataPath,'model','aircraft','ase','models', ACModelVersion);

Aquila = build_aircraft(ACModelVersion, ACModelBuild, model_directory);  

ialt            = find(ismember(Aquila.linear.index.alt, alts)) ;
ieas            = find(ismember(Aquila.linear.index.eas, eass)) ;
igamma          = find(ismember(Aquila.linear.index.gamma, gammas)) ;
iphi            = find(ismember(Aquila.linear.index.phi, phis)) ;
idelta_spoiler  = find(ismember(Aquila.linear.index.phi, delsps)) ;

state_0   = Aquila.linear.trim(ialt, ieas, igamma, iphi, idelta_spoiler).x;
u_0       = Aquila.linear.trim(ialt, ieas, igamma, iphi, idelta_spoiler).u;


[y_disp, y_vel, y_accel, y_rel, y_control, y_propulsion]    = Aquila.nonlinear.compute_output(state_0, u_0);

%           1st element: alt index
%           2nd element: eas index
%           3rd element: gamma index
%           4th element: bank index
%           5th element: spoiler index

trim_idx = {ialt, ieas, igamma, iphi,idelta_spoiler } ;

disp('.........');
disp('..........');
disp('Selected flight condition is:');
% print the details of the trim point
Aquila.print_trim_point(trim_idx{:});



disp('..........');
disp('determine an input-output pair for the analysis from the tables: ');
disp('..........');


for jj=1:size(AquilaModelInputNames,2)
    inputput_index_str{jj,1} = num2str(jj) ;
end

Control_inputs = AquilaModelInputNames' ;

Input_Table = array2table(Control_inputs,'RowNames',inputput_index_str);
disp(Input_Table) ;
% Determine the desired input-output pair for the root locus analysis

u_idx = input(' Enter an input index from the input list :  ','s');
if isempty(u_idx)
  u_idx = '1';
  disp('   A valid input index entry is required ');
end

disp('..........');
disp('..........');

% Determine the desired input-output pair for the root locus analysis  
for jj=1:size(AquilaModelOutputNames,2)
    output_index_str{jj,1} = num2str(jj) ;
end

Output_signals = AquilaModelOutputNames' ;
Output_Table   = array2table(Output_signals,'RowNames',output_index_str) ;

disp(Output_Table) ;

y_idx = input(' Enter an output index from the output list :  ','s');
if isempty(y_idx)
  y_idx = '16';
  disp('   A valid output index entry is required ');
end

input_idx  = str2double(u_idx) ;
output_idx = str2double(y_idx) ;

disp('..........');
disp(['Selected input is ',char(Control_inputs(input_idx)),' and selected output is ', char(Output_signals(output_idx)), ' measured at node location ', node_str]);
disp('..........');
disp(' Check the sign for the transfer functions used in the root locus study ...  ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For the release revision, add a feature to enable accepting the "sys" from user 
[sys] = ase_lin_sys_gen(Aquila,trim_idx, output_at_node , node_idx , 0 ,torque_const,throttle_mixing) ;  %torque_const=45.2

for i=1:length(Aquila.definition.state.label)
    state_labels(i) = Aquila.definition.state.label{i};
    state_lags(i)   = strncmp('lag',state_labels(i),3);
end

sys.InputName  = AquilaModelInputNames ;
sys.OutputName = AquilaModelOutputNames ;    
sys.StateName  = state_labels ;

alt_str = num2str(Aquila.linear.index.alt(ialt)) ;
eas_str = num2str(Aquila.linear.index.eas(ieas)) ;
eas_str = strrep(eas_str,'.','p');

if ( saveFltCnds == true )
    %record the desired flight condition with the proper name
    disp('..........');
    disp('Saving the aircraft linearized model at the given flight condition in Aquila_OL_Model.mat ....');
    save(['Aquila_OL_Model_',alt_str,'_',eas_str],'trim_idx', 'Aquila','sys','output_at_node','alts','eass','gammas','phis','ACModelDataPath','ACModelBuild','ACModelVersion');
end

sys_full = sys(output_idx,input_idx) ;

%residualize the model - Quasi-steady aerodynamic model
% sys_modred_residual = modred(sys,state_lags, 'matchdc');
sys_modred_residual = modred(sys_full,state_lags, 'matchdc');   %perform the modred command on the specific input-output pair
figure;
pzmap(sys_modred_residual);xlim([-15,2]);ylim([-0.5,50]);sgrid;
title(['system Pole and zero locations at flight condition ', num2str(Aquila.linear.index.alt(ialt)), ' (m) and ', num2str(Aquila.linear.index.eas(ieas)), ' (m/s) ']);
legend('Residualized (Aero Lags)');

%truncate the inertial position states
%Also truncate the 3 first inertial position states:
elim=(1:3);
sys_modred_residual = modred(sys_modred_residual,elim,'truncate');

sys_modred_poles = eig(sys_modred_residual.A);
sys_full_poles   = eig(sys_full.A);

% magnitude of the minreal_Tol can change the sys_minreal behaviour -
% before further analysis, behaviour of the sys_minreal system should be
% cross-checked against the full-order model
minreal_Tol = 1e-10; % Set tolerance (increasing it, will force additional cancellations)
sys_minreal = minreal(sys_full,minreal_Tol, false);
disp('..........');

%root loci for the system from the elevator to the Nz
figure;rlocusplot((-1)*sys_modred_residual);grid; xlim([-10,5]);ylim([0,45]);title(['Root locus- ',node_str ,' to Elevator (residualized)']);
figure;rlocusplot((-1)*sys_full);grid; xlim([-10,5]);ylim([0,45]);title(['Root locus- ',node_str ,' to Elevator (full order)']);

figure;rlocusplot((-1)*sys_minreal);grid; xlim([-10,5]);ylim([0,45]);title(['Root locus- ',node_str ,' to Elevator (minimal realization)']); 

%compare locations of the poles for the full-order model and the reduced order model  
figure;
h1 = plot(real(sys_full_poles), imag(sys_full_poles),'bx','MarkerSize',12); 
hold on;
h2 = plot(real(sys_modred_poles), imag(sys_modred_poles),'rx','MarkerSize',12);xlim([-15,2]);ylim([0,50]);ylabel('Imag, (rad/sec)');xlabel('Real, (1/sec)');
title(['Aquila Pole locations at flight condition ', num2str(Aquila.linear.index.alt(ialt)), ' (m) and ', num2str(Aquila.linear.index.eas(ieas)), ' (m/s) ']);
sgrid;
h_leg = legend([h1 h2],'Full Order', 'Residualized (Aero Lags)');
h_leg.Location = 'SouthWest';

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


if ( printAirplaneConfigurationInfo == true )

    % CG location
    disp('..........');
    disp('The aircraft CG location (in m) relative to the aircraft nose in the x-aft, z-up frame (centered at the nose)');
    disp(num2str(Aquila.configuration.CG));

    disp('.........');
    disp('Airplanes mass (in kg) is:');
    disp(Aquila.configuration.mass);

    disp('..........');
    disp('Airplanes inertia (kg.m^2) is:');
    disp(Aquila.configuration.inertia);

    
    % The corresponding data hasn't been included into the model files -
    % it's hardcoded for now
    disp('..........');
    disp('Elevons inner hinge location (in m) relative to the aircraft nose in the x-aft, z-up frame (centered at the nose):');
    disp([+7.56264, 17.97, +1.06373]);
    disp('.........');
    disp('Elevons outer hinge location (in m) relative to the aircraft nose in the x-aft, z-up frame (centered at the nose):');
    disp([+8.54977, 20.7989, +1.21534]);

    disp('..........');
    disp('The node locations relative to the aircraft nose in the x-fwd, z-down frame (centered at the nose):');
    disp('Displacement of the nodes are relative to the mean axis frame; nodes absolute (6DOF) velocity and acceleration are available');
    disp('Note that the set of sensor nodes is fixed for each configuration');
    disp(Aquila.definition.node)

    disp('..........');
    disp('Propellers locations are relative to the aircraft nose in the x-aft, z-up frame (centered at the nose):');
    disp(Aquila.nonlinear.positions)

    disp('..........');
    disp('The elastic modes states are states 13:26');
    disp('The elastic mode state derivatives are states 27:40');
    disp('The aerodynamic lag states are states 41:80, modes have been labeled with the elastic mode they correspond to ...');
    disp('..........');
    disp('The complete list of the states are:');
    for i=1:length(Aquila.definition.state.label)
        disp(Aquila.definition.state.label{i});
    end
    disp('.........');         
    disp('With the following definitions for the states ... :');
    disp(['p_N (m), p_E (m), h (m), phi (rad), theta (rad), psi (rad), V_T (m/s), beta (rad), alpha (rad), P (rad/s), Q (rad/s), R (rad/s), EL_7,right_elevon, EL_8,left_elevon, EL_9,antisymmetric_bending_1, EL_10,symmetric_bending_1']);
    disp('.........');
    disp(['EL_11,symmetric_bending_2, EL_12,symmetric_torsion_1, EL_13,antisymmetric_bending_2, EL_14,antisymmetric_torsion_1, EL_15,symmetric_bending_3, EL_16,antisymmetric_elevon_bending_1, EL_17,symmetric_elevon_bending_1, EL_18,symmetric_torsion_2'])
    disp('.........');
    disp(['EL_19,antisymmetric_torsion_2, EL_20,antisymmetric_bending_3, EL_dot_7, EL_dot_8, EL_dot_9, EL_dot_10, EL_dot_11, EL_dot_12, EL_dot_13, EL_dot_14, EL_dot_15, EL_dot_16, EL_dot_17, EL_dot_18, EL_dot_19, EL_dot_20']);
    disp('.........');
    
    % Trim thrust and Torque 
    disp('.........');
    Torque_   = u_0(6:9)' ;
    disp('Trim motor torques (in N.m for each motor) - descending order from the outer left motor  ');
    disp('Positive sign for CW rotation viewing from the rear');
    disp(num2str(Torque_'));

    disp('.........');
    Thrust_   = y_propulsion(17:20) ;
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
    
    disp('.........');
    disp('.........');
end

if( plotModes == true )
    % create ploter and add the Aircraft objects as sources for eigenvaules
    plotter = EigenvaluePlotter();
    plotter.add_source(Aquila.version, Aquila);
    plotter.nmodes_label = 4 ;

    % plot a comparison
    % names = {aircraft_1601.version, aircraft_1601_Pre2.version};
    names = {Aquila.version};

    figure;
    plotter.show_modes = true;
    plotter.plot_comparison(names, trim_idx);
    xlim([-8., 1.]);
    ylim([-1., 50.]);

    figure;
    plotter.freq_high = 2.;
    plotter.plot_comparison(names, trim_idx);
    xlim([-8., 1.]);
    ylim([-1., 2. * 2 * pi()]);
end


if( plotUnstableModes == true )
    if ( printFlutterOnlyBoundry == true )
        % Plot the flutter boundry (plot flutter boundry only)
        test_unstable_modes(Aquila, model_directory,'flutter_only');
    else
        %Plot the stability boundry (all modes)
        test_unstable_modes(Aquila, model_directory);
    end
end


figure;plot3(Aquila.nonlinear.nodes_plot(:,1),Aquila.nonlinear.nodes_plot(:,2),Aquila.nonlinear.nodes_plot(:,3),'r.');
grid on; box;xlim([-15,15]);ylim([-22,22]);zlim([-10,10]);title('Aquila airplane undeformed shape');xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');

disp('.........');
disp('.........');
reply = input('Do you want to see the flexible mode shapes? Y/N [Y]:','s');
if isempty(reply)
  reply = 'Y';
end

if ( reply == 'Y' )

    %Plot the mode shapes:
    %This modeshape matrix is 3 dimensional. The shape is <number of modes, number of nodes, 6>. 
    %The locations of all of the nodes is aircraft.nonlinear.nodes_plot.
    mode_shape_matrix = Aquila.nonlinear.phi_T_plot ;
    mode_shape_scalar = 15; 

    x_mode_shapes = mode_shape_scalar*mode_shape_matrix(:,:,1) ;
    y_mode_shapes = mode_shape_scalar*mode_shape_matrix(:,:,2) ;
    z_mode_shapes = mode_shape_scalar*mode_shape_matrix(:,:,3) ;
    rx_mode_shapes = mode_shape_matrix(:,:,4) ;
    ry_mode_shapes = mode_shape_matrix(:,:,5) ;
    rz_mode_shapes = mode_shape_matrix(:,:,6) ;

    marker_size  = 6 ;
    %choose the mode shape number to be plotted 
    for i_mode = 1:size(mode_shape_matrix,1)

       % color map based on the vertical deflections (tranlational displacement across the z_body axis): 
       disp_color_map =  (Aquila.nonlinear.nodes_plot(:,3)'+z_mode_shapes(i_mode,:)) ;
       % color map based on the twist angles (angular deflection around the y_body axis): 
       % rot_color_map =  mode_shape_scalar*(ry_mode_shapes(i_mode,:)) ;                                 

       color_map_base = repmat([0.5 0.5 0.5],length(Aquila.nonlinear.nodes_plot(:,3)),1) ;
       figure;
       % plot3((Aquila.nonlinear.nodes_plot(:,1)'+x_mode_shapes(i_mode,:)),(Aquila.nonlinear.nodes_plot(:,2)'+y_mode_shapes(i_mode,:)),(Aquila.nonlinear.nodes_plot(:,3)'+z_mode_shapes(i_mode,:)),'r.');
       scatter3((Aquila.nonlinear.nodes_plot(:,1)'+x_mode_shapes(i_mode,:)),(Aquila.nonlinear.nodes_plot(:,2)'+y_mode_shapes(i_mode,:)),(Aquila.nonlinear.nodes_plot(:,3)'+z_mode_shapes(i_mode,:)),marker_size,disp_color_map);
       colormap(jet);colorbar;
       hold on;
       scatter3(Aquila.nonlinear.nodes_plot(:,1),Aquila.nonlinear.nodes_plot(:,2),Aquila.nonlinear.nodes_plot(:,3),marker_size,color_map_base);
       grid on; box;xlim([-15,15]);ylim([-22,22]);zlim([-10,10]);title(['Aquila mode shape number ',num2str(i_mode)]);xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
    end

end
  
if (plotSpdAltLoci==true) 
    

    % compare_eigenvalues(Aquila, Aquila, alts, eass, gammas, phis)

    % create plotter and add the Aircraft objects as sources for eigenvaules
    color_1601 = [0., 0., 1.];
    colormap_compare = [color_1601];

    plotter1 = EigenvaluePlotter();
    plotter1.add_source(Aquila.version, Aquila, colormap('bone'));

    plotter1.marker_size = 5;
    plotter1.colormap = colormap_compare;
    plotter1.show_freq = true;
    plotter1.use_sgrid = true;
    xlim_ = [-2., 1.];
    ylim_ = [-1., 8.];
    xlim_close = [-.5, .25];
    ylim_close = [-.2, 2.];

    %% plot a sweep on velocity
    index = {ialt, ieas, igamma, iphi, idelta_spoiler};
    Aquila.print_trim_point(index{:});
    fig = figure();
    % names = {aircraft_1A_1.version, aircraft_1501C.version, aircraft_1601.version};
    names = {Aquila.version};
    sweep_variable = 'eas';
    indicies_sweep     = 1:length(Aquila.linear.index.eas);
    plotter1.show_modes = false;
    plotter1.freq_high  = Inf;
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_, ylim_);
    directory = pwd ;
    fig_name = fullfile(directory, 'vel_sweep.png');
    save_fig(fig, fig_name);
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_close, ylim_close);
    fig_name = fullfile(directory, 'vel_sweep_close.png');
    save_fig(fig, fig_name);

    %% plot a sweep on bank angle
    fig = figure();
    % names = {aircraft_1A_1.version, aircraft_1501C.version, aircraft_1601.version};
    names = {Aquila.version};
    sweep_variable = 'phi';
    indicies_sweep = 1:length(Aquila.linear.index.phi);
    plotter1.show_modes = false;
    plotter1.freq_high = Inf;
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_, ylim_);
    fig_name = fullfile(directory, 'bank_sweep.png');
    save_fig(fig, fig_name);
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_close, ylim_close);
    fig_name = fullfile(directory, 'bank_sweep_close.png');
    save_fig(fig, fig_name);

    %% plot a sweep on flight path angle
    fig = figure();
    names = {Aquila.version};
    sweep_variable = 'gamma';
    indicies_sweep = 1:length(Aquila.linear.index.gamma);
    plotter1.show_modes = false;
    plotter1.freq_high = Inf;
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_, ylim_);
    fig_name = fullfile(directory, 'gamma_sweep.png');
    save_fig(fig, fig_name);
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_close, ylim_close);
    fig_name = fullfile(directory, 'gamma_sweep_close.png');
    save_fig(fig, fig_name);

    %% plot a sweep on altitude
    fig = figure();
    % names = {aircraft_1A_1.version, aircraft_1501C.version, aircraft_1601.version};
    names = {Aquila.version};
    sweep_variable = 'alt';
    indicies_sweep = 1:length(Aquila.linear.index.alt);
    plotter1.show_modes = false;
    plotter1.freq_high = Inf;
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_, ylim_);
    fig_name = fullfile(directory, sprintf('alt_sweep_eas_%.2f.png', Aquila.linear.index.eas(index{2})));
    save_fig(fig, fig_name);
    plotter.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_close, ylim_close);
    fig_name = fullfile(directory, sprintf('alt_sweep_eas_%.2f_close.png', Aquila.linear.index.eas(index{2})));
    save_fig(fig, fig_name);

    %% plot a sweep on altitude
    index{2} = 5;
    Aquila.print_trim_point(index{:});
    fig = figure();
    % names = {aircraft_1A_1.version, aircraft_1501C.version, aircraft_1601.version};
    names = {Aquila.version};
    sweep_variable = 'alt';
    indicies_sweep = 1:length(Aquila.linear.index.alt);
    plotter1.show_modes = false;
    plotter1.freq_high = Inf;
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_, ylim_);
    fig_name = fullfile(directory, sprintf('alt_sweep_eas_%.2f.png', Aquila.linear.index.eas(index{2})));
    save_fig(fig, fig_name);
    plotter1.plot_sweep(names, index, sweep_variable, indicies_sweep, xlim_close, ylim_close);
    fig_name = fullfile(directory, sprintf('alt_sweep_eas_%.2f_close.png', Aquila.linear.index.eas(index{2})));
    save_fig(fig, fig_name);
end

%% Modal Controllability/Observability and Impulse Residue analysis for the selected modes
if( perfomCtrbObsvAnalysis == true )
    disp('.........');
    disp('.........');
    reply = input('Use the residualized model for the controllability/observability analysis? Y (for reduced model - recommended), N (for full model): ','s');
    if isempty(reply)
      reply = 'Y';
    end

    if (reply == 'Y')
        sys_co = sys_modred_residual ;
        disp('  Using the residualized reduced order model ...') ;
    else
        sys_co = sys_full;
        disp(' Using the full order model ... '); 
    end



    %Test the controllability
    CO = ctrb(sys_co.A, sys_co.B);
    svd_CO = svd(CO);
    %default tol is tol = max(size(CO))*eps(max(svd_CO))
    r_CO = sum(svd_CO >= 1e-6);                      %calculate the rank of the controllability matrix
    very_small_svd_CO = find(svd_CO <= 1e-06);

    disp('.........');
    disp('.........');
    disp('Controllability matrix rank') 
    disp(r_CO);

    if isempty(very_small_svd_CO)
        disp('System Controllable (CO matrix svd analysis) ...');
    else
        disp('System NOT Controllable');
        %See if the system is stabilizable:
        P1 = eig(sys_co.A) ;

        K2 = 2*eye(size(sys_co.B,2),size(sys_co.A,1)) ;  % check the dimensions
        P2 = eig(sys_co.A - sys_co.B * K2) ;

        uncontrollable_modes = find(abs(P1-P2)<= 1e-06) ;
    end

    %write it in the controllability Staircase form 
    [Ab,Bb,Cb,T,k] = ctrbf(sys_co.A,sys_co.B,sys_co.C,1e-06) ;

    disp('>>>>>>>>>>>>>>>>>>>>>>>>>>>>');
    disp('Number of controllable modes - from the controllability staircase form (might not produce same results as the previous svd based analysis) ');
    disp('>>>>>>>>>>>>>>>>>>>>>>>>>>>>');
    disp(sum(k));

    %Evaluate degrees of controllability:
    %Get the Eigen structure: eigen values and right/left eigen vectors
    %W whose columns are corresponding left eigenvectors so that W'*A = D*W' 
    %V whose columns are the corresponding right eigenvectors so that A*V = V*D
    [V,D,W] = eig(sys_co.A);

    %check the numerical opeations accuracy:
    diff_mat_1 = (W'*sys_co.A*V) - D ;
    [rows_1,cols_1,vals_1] = find(abs(diff_mat_1)>10) ;

    %check the Left eigenvectors accuracy:
    diff_mat_2 = (W'*sys_co.A-D*W');
    [rows_2,cols_2,vals_2] = find(abs(diff_mat_2)>1e-6);

    %Right eigenvectors
    diff_mat_3 = (sys_co.A*V-V*D);
    [rows_3,cols_3,vals_3] = find(abs(diff_mat_3)>1e-6);

    diff_mat_4 = (inv(V)*sys_co.A*V) - D ;
    [rows_4,cols_4,vals_4] = find(abs(diff_mat_4)>10) ;

    %%% reduce model size
    fmax=3;  %reduced model Hz 
    [sys_modal,T]=canon(sys_co,'modal', 1e12);  % Similarity transformation and canonical form
    A_modal = sys_modal.a;

    %Degree of controllability between airplane modes and the first input input_idx ( sys_co.B(:,1) )
    %use the inverse of right eigenvector matrix for the controllability study
    V_inv = inv(V) ;
    diff_mat_5 = (T - V_inv) ;
    [rows_5,cols_5,vals_5] = find(abs(diff_mat_5)>1e-3) ;

    for ii=1:size(V,1)
        % Degree of Controllability of the ii'th mode to the control syrface determined by "input_idx"
        % elements of the DoC are cosine of the angle between the ii'th mode
        % and the control input vector corresponding to the jj'th control signal
        DoC_cosAngle(ii,1) = dot(V_inv(ii,:),sys_co.B(:,input_idx)) / (norm(V_inv(ii,:))*norm(sys_co.B(:,input_idx))) ;
        DoC(ii,1) = dot(V_inv(ii,:),sys_co.B(:,input_idx)) ;

    end

    %number of modes on which the elevator has the largest degree of
    %controllability:
    number_modes = 10 ; 
    [~, isort] = sort(abs(DoC), 'descend');
    contributing_modes = sys_co.StateName(isort(1:number_modes));


    figure;
    h3=plot(real(DoC), imag(DoC),'rd','MarkerSize',12);
    hold on;

    % ylabel('Imag, (rad/sec)');xlabel('Real, (1/sec)');grid;
    % title(['Degree of Controllability from the elevator command at flight condition ', num2str(Aquila.linear.index.alt(ialt)), ' (m) and ', num2str(Aquila.linear.index.eas(ieas)), ' (m/s) ']);
    % h_leg = legend(h3,'');
    % h_leg.Location = 'NorthEast';

    figure(1003);title(['Degree of Controllability from elevator input to the modes measured at ',node_str]); grid;hold on;
    for i = 1:number_modes

        plot(real( DoC(isort(i))), imag( DoC(isort(i))) ,'r>-','MarkerSize',12);

        x = real( DoC(isort(i)) ) + 1.5;
        y = imag( DoC(isort(i)) ) + 5;
        label_formatted = format_label(contributing_modes{i}); 
        label_formatted = strcat('$',label_formatted,' \leftarrow \delta_e ','$');
        text(x, y , label_formatted,'Fontsize',12,'interpreter','latex');
        hold on;    
    end

    %show that the the eigenvectors corrosponding to the complex conjugate
    %eigen values are also complex conjugate: (compare it for the left eigen
    %vectors)
    % diff_vec = (ctranspose(W(:,1)) - transpose(W(:,2)));
    % find(abs(diff_vec)>1e-6)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Test the observability
    OB = obsv(sys_co.A, sys_co.C);  %6216*74
    svd_OB = svd(OB);
    %default tol is tol = max(size(CO))*eps(max(svd_CO))
    r_OB = sum(svd_OB >= 1e-6);   %calculate the rank of the observability matrix
    very_small_svd_OB = find(svd_OB <= 1e-06);

    disp('.........');
    disp('.........');
    disp('Observability matrix rank') 
    disp(r_OB);

    %Partition the system with the stabsep command and see if the unstable
    %modes are observable.

    if isempty(very_small_svd_OB)
        disp('System Observable');
    else
        disp('System NOT Observable');
        %See if the system is detectable:
        P1 = eig(sys_co.A) ;

        L2 = 2*eye(size(sys_co.A,1),size(sys_co.C,1)) ;  %check the dimensions
        P2 = eig(sys_co.A - L2 * sys_co.C) ;

        unobservable_modes = find(abs(P1-P2)<= 1e-06) ;

    end

    %write it in the observability Staircase form 
    [Ab,Bb,Cb,T,k] = obsvf(sys_co.A,sys_co.B,sys_co.C,1e-06) ;

    disp('>>>>>>>>>>>>>>>>>>>>>>>>>>>>');
    disp('  Number of observable modes');
    disp('>>>>>>>>>>>>>>>>>>>>>>>>>>>>');
    disp(sum(k));


    disp('.........');
    disp('.........');
    %Perform the residue study by using the full-order model
    disp('   Perform impulse response (residue method) analysis by using the chosen (system) model')

    [V_,D_,W_] = eig(sys_co.A);
    V_inv = inv(V_) ;

    Aquila_modes = diag(D_);

    for jj=1:size(sys_co.A,1)
        mode_index_str{jj,1} = num2str(jj) ;
    end

    disp('.........');
    Modes_Table = array2table(Aquila_modes,'RowNames',mode_index_str) ;
    disp(Modes_Table) ;

    % Calculate the impulse residues for an unstable Aeroelastic mode from a given input-output pair 
    % Enter the mode index first for which the impulse residue study is
    % performed - example for cell array of char vectors would be 26 or [26 27]
    mode_idx = input(' Based on the modes table above - enter mode(s) indeces (in a vector as 26 or [26 28]) for impulse residues calculations :  ');

    if isempty(mode_idx)
      mode_idx = 20;
      disp('   A valid mode index entry is required ');
    end

    for i_idx = 1:length(mode_idx)
        Impulse_Residue(i_idx,1)     = abs(   ( sys_co.C*V_(:, mode_idx(i_idx)) ) * ( V_inv(mode_idx(i_idx),:)*sys_co.B ) ) ;
        Modal_Obsv(i_idx,1) = abs( sys_co.C*V_(:, mode_idx(i_idx)) )  ;
        Modal_Ctrb(i_idx,1) = abs( V_inv(mode_idx(i_idx),:)*sys_co.B ) ;
    end

    disp('>>>>>>>>>>>>>>>>>>>');
    disp(['   impulse residues for the chosen modes ... ',node_str,' (results are printed into a csv file with the node name)']);
    disp('>>>>>>>>>>>>>>>>>>>');
    Mode_Index = mode_idx' ;
    Res_AE_Table = table(Mode_Index,Modal_Obsv,Modal_Ctrb, Impulse_Residue) ;
    disp(Res_AE_Table) ;

    %Print the impulse residue study results and the
    %controllability/observability metrics of the selected modes into a csv document
    disp(['Selected input is ',char(Control_inputs(input_idx)),' and selected output is ', char(Output_signals(output_idx)), ' measured at node location ', node_str]);

    Mode_residue_file_name = [char(Control_inputs(input_idx)),'_to_',char(Output_signals(output_idx)),'_at_',node_str,'.xlsx'];
    Mode_residue_file_name = strrep(Mode_residue_file_name,' ','_');
    writetable(Res_AE_Table,Mode_residue_file_name);
end

%% Plot transfer functions - This section is designed to plot transfer functions for different input-output and nodes pairs:
if ( plotTransferFunctions == true )

    plotter = TransferFunctionPlotter();
    plotter.f_list = logspace(-2, 1, 1000);
    plotter.show_figure = true;
    plotter.save_to_file = false;

    plotter.add_source(Aquila.version, Aquila,  [0., 0., 1.], '-');

    %% Plot particular loops of interest for a single model
    plotter.path_output_base = pwd ;

    i_input  = Aquila.index.input.delta_c;                    % Index of input parameter.
    i_output = Aquila.index.output_control.delta_r;           % Index of output parameter.
    otype = 'control';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

    % determine the aircraft node numbers and sequence from the
    % 'ase_lin_sys_gen.m' file - the following indices should be an exact
    % copy of the sequence in the 'ase_lin_sys_gen.m' file
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

    i_input  = Aquila.index.input.delta_c;           % Index of input parameter.
    i_output = Lwingtip_trans_idx(3);                % Index of output parameter.
    otype = 'accel';                                 % Output type descriptor : str ['state', 'disp', 'vel', 'accel', 'rel', 'control', 'thrust']
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

    i_input  = Aquila.index.input.delta_c;
    i_output = Rwingtip_trans_idx(3);
    otype = 'accel';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

    i_input  = Aquila.index.input.delta_c;
    i_output = LI_trans_idx(3);
    otype = 'accel';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

    i_input  = Aquila.index.input.delta_c;
    i_output = RI_trans_idx(3);
    otype = 'accel';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);


    i_input  = Aquila.index.input.delta_c;
    i_output = LO_trans_idx(3);
    otype = 'accel';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

    i_input   = Aquila.index.input.delta_c;
    i_output  = RO_trans_idx(3);
    otype = 'accel';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

    i_input  = Aquila.index.input.delta_c;
    i_output = IMU_trans_idx(3);
    otype = 'accel';
    plotter.plot_input_to_output_loop(Aquila.version, trim_idx, i_input, i_output, otype);

end


%% Preparation for the siso design tool - control loops structure determination
if ( useSISOtool == true )
    
    %Use the SISO design tool to find the control laws structure,
    %Put the compensator in the feedback path with negative feedback sign
    % controlSystemDesigner(((-1/g_acc)*sys_minreal(output_idx,input_idx)),'LocationFlag','feedback','FeedbackSign','-1');
    figure;step(sys_full*Deg2Rad);  %checking the sign

    %first guess for the controller
    % C = zpk([-1.8, -15+10*1i, -15-10*1i],[-18, -35+5*1i, -35-5*1i],0.08);
    % Set the loop configuration for the most inner loops control 

    % choosing configuration #2 where the sensors and the controller are in the feedback loop - negative feedback
    disp('   Calling the SISO design tool by using the minimal realization model')

    T = sisoinit(2);
    % Plant:
    T.G.Value = ((-1)*sys_minreal) ;
    % Initial Guess for the controller
    T.C.Value = zpk(-1.8,-18,0.08) ;

    % Views for tuning Open-Loop OL1.
    T.OL1.View = {'rlocus','nichols'}; 

    % Launch SISO Design Tool using configuration T
    controlSystemDesigner(T)
end
