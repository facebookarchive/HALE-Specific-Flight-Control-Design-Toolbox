close all; clear; clc;


% Load sysID system

sys_sysID = load('../../CustomModels/StateSpace/ss_0_9id.mat');
sys_sysID = sys_sysID.sys;
sys_sysID.Name = 'n4sid_EAS_9';

% Get ROM system
sys = load(['../../CustomModels/StateSpace/ss_0_9rom.mat']);
sys = sys.sys;

%% Augment ROM
A_new = sys.A;
B_new = sys.B;
C_new = sys.C;
D_new = sys.D;

% Augment with new states / replace bad position with integrated
% velocity
A_new = [A_new; C_new([4:6,9],:)]; % Augment new p_N, p_E, press_alt, psi
A_new = [A_new zeros(size(A_new,1),4)]; % Add zeros to account for new states
B_new = [B_new; D_new([4:6,9],:)]; % Augment new p_N, p_E, press_alt, psi

C_new = [C_new zeros(size(C_new,1),4)]; % Add zeros to account for new states
C_new([10:12,15],:) = 0; % Clear old outputs
C_new(10,end-3) = 1; % Add new p_N output
C_new(11,end-2) = 1; % Add new p_e output
C_new(12,end-1) = 1; % Add new pres_alt output
C_new(15,end) = 1; % Add new psi output

D_new([10:12,15],:) = 0; % Clear old passthroughs 

sys_aug = ss(A_new,B_new,C_new,D_new);
sys_aug.InputName = sys.InputName;
sys_aug.OutputName = sys.OutputName;
sys_aug.Name = sys.Name;

% %% System Poles
% aswingEigs = load('./data/EAS9_aswingEigs.mat');
% aswingEigs = aswingEigs.eigs;
% figure
% pzmap(sys_aug); hold on
% pzmap(sys_sysID);
% pzmap([],aswingEigs);
% grid

%% Bode
% fr_data_nonlin = load('../AQ3nonlin.bode');
% fr_data_nonlin = fr_data_nonlin.fr_data_nonlin;

in = {'Aileron','Elevator','Rudder','Motor'};
out = {'dUx','dUy','dUz','UX','UY','UZ','Wx','Wy','Wz','RX','RY','RZ','Phi','Theta','Psi','V','beta','alpha'};

% Open bode file
fid2 = fopen('../AQ3nonlin.bode');

Response2 = [];

% Parse Bode information from Aswing
for u = 1:numel(in)
    for y = 1:numel(out)
        % Read Aswing Nonlinear Frequency Response
        R = textscan(fid2,'%f %f %f',120,'CommentStyle','#','Delimiter','\t');
        R = [R{:}];
        Freq2 = R(:,1);
        Gain2 = R(:,2);
        Phase2 = R(:,3);
        % Convert response to complex from
        % rho e^(J theta) = rho * cos(theta) + j rho * sin(theta)
        Response2(y,u,:) = Gain2.*cos(deg2rad(Phase2))+1i*Gain2.*sin(deg2rad(Phase2));
    end 
end

% Convert frequency from Hz to rad/s
Freq2 = Freq2 * (2 * pi());

% Create frequency response object.
fr_data_nonlin = idfrd(Response2,Freq2,0);
fr_data_nonlin.InputName = in;
fr_data_nonlin.OutputName = out;
fr_data_nonlin.Name = 'ASWING Nonlinear';

pairs = {{'Elevator','Wy'},
        {'Elevator','Theta'},
        {'Aileron','Wx'},
        {'Rudder','Wz'},
        {'Elevator','V'},
        {'Motor','V'},
        {'Elevator','RZ'},
        {'Motor','RZ'},
        {'Motor','dUx'},
        {'Motor','dUz'}};
options = bodeoptions;
options.PhaseWrapping = 'on';
options.Grid = 'on';
minFreq = fr_data_nonlin.Freq(1);
maxFreq = fr_data_nonlin.Freq(end);
for n=1:numel(pairs)
   figure
   k = pairs{n}{2};
   j = pairs{n}{1};
   ffplot(fr_data_nonlin(k,j)); hold on
   bode(sys_aug(k,j),{minFreq,maxFreq},options); 
   bode(sys_sysID(k,j),{minFreq,maxFreq},options); 
   legend('Location','Best')
end

prettyPlots()

% figHandles = get(groot,'Children');
% for i = 1:numel(figHandles)
%    figure(figHandles(i));
%    export_fig ROM_SysID_Nonlinear.pdf -append
% end