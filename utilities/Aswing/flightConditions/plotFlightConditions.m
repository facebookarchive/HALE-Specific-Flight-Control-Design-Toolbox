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
%   Developer:  Abe Martin
close all; clear all;

% Flight speeds
easList = [7.5 8.5 9.5 10.5];

for i = 1:numel(easList)
    
    eas = easList(i);
   
    % First get list of data files associated with this speed
    files = dir(['./data/EAS' num2str(easList(i)) '_*.mat']);
    
    % Get ROM system
    files(11).name
    sys = load(['./data/' files(11).name]);
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
    
    %% System Poles
    aswingEigs = load(['./data/' files(1).name]);
    aswingEigs = aswingEigs.eigs;
    figure(1)
    pzmap(sys_aug); hold on
    grid
    figure(2)
    pzmap(aswingEigs,[]); hold on
    grid
    figure(3)
    pzmap(sys_aug); hold on
    pzmap([],aswingEigs);
    grid
    
    %% Bode
    fr_data_nonlin = load(['./data/' files(10).name]);
    fr_data_nonlin = fr_data_nonlin.fr_data_nonlin;
    
    pairs = {{'Elevator','Wy'},
            {'Elevator','Theta'},
            {'Aileron','Wx'},
            {'Rudder','Wz'},
            {'Elevator','V'},
            {'Motor','V'},
            {'Elevator','RZ'},
            {'Motor','RZ'},
            {'Motor','dUx'},
            {'Motor','dUz'},
            {'Rudder','Psi'},
            {'Rudder','RY'},
            {'Rudder','UY'}};
    options = bodeoptions;
    options.PhaseWrapping = 'on';
    options.Grid = 'on';
    minFreq = fr_data_nonlin.Freq(1);
    maxFreq = fr_data_nonlin.Freq(end);
    for n=1:numel(pairs)
       figure(4+n)
       k = pairs{n}{2};
       j = pairs{n}{1};
       ax = gca;
       ax.ColorOrderIndex = i;
       ffplot(fr_data_nonlin(k,j)); hold on
       ax = gca;
       ax.ColorOrderIndex = i;
       bode(sys_aug(k,j),'--',{minFreq,maxFreq},options); 
       legend('show','Location','Best')
    end
    
    %% Time Domain Plots
    trim = load(['./data/' files(12).name]);
    trim = trim.data;
    
    % Plot cm_alpha
    
%     
%     out = {'dUx','dUy','dUz','UX','UY','UZ','Wx','Wy','Wz','RX','RY','RZ','Phi','Theta','Psi','V','beta','alpha'};
%     out_aswing = {'dUxLacc1','dUyLacc1','dUzLacc1','UXvel1','UYvel1','UZvel1','Wxrate1','Wyrate1','Wzrate1','RXpos1','RYpos1','RZpos1','Phi1','Theta1','Psi1','V1','beta1','alpha1'};
%     
%     % Nonlinear Trim
%     for j = 1:numel(out)
%        figure(15+j)
%        plot(trim.t,trim.(char(out_aswing(j))),'DisplayName',['EAS ' num2str(eas)]); hold on
%        title(['Aswing Trim Conditions ' out(j)])
%        xlabel('Time (s)')
%        legend('show','Location','Bestoutside')
%        grid
%     end
%     
%     % Motor Up to Outputs
%     timedata = load(['./data/' files(9).name]);
%     timedata = timedata.data;
%     timedata_motorup = timedata;
%     opt = stepDataOptions;
%     opt.StepAmplitude = 0.01;
%     for j = 1:numel(out)
%         figure(15+18+j)
%         % ROM Time response
%         [y,t] = step(sys_aug(out(j),'Motor'),120,opt);
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(t,y,'--','DisplayName',['EAS ' num2str(eas) ' ROM']); hold on
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(timedata.t,timedata.(char(out_aswing(j)))-timedata.(char(out_aswing(j)))(1),'DisplayName',['EAS ' num2str(eas) ' Aswing']); hold on
%         title(['Motor + 0.01 to ' out(j)])
%         xlabel('Time (s)')
%         grid
%         legend('show','Location','Bestoutside')
%     end
%     
%     % Motor Up to Twist
%     twist_init_right = timedata.Theta2(1)-timedata.Theta4(1);
%     twist_init_left = timedata.Theta3(1)-timedata.Theta4(1);
%     figure(15+18+18+1)
%     plot(timedata.t,(timedata.Theta2-timedata.Theta4),'DisplayName',['Right Wing EAS ' num2str(eas)]); hold on
% %     plot(timedata.t,(timedata.Theta3-timedata.Theta4),'DisplayName',['Left Wing EAS ' num2str(eas)]); hold on
%     title('Motor + 0.01 to Twist')
%     xlabel('Time (s)')
%     ylabel('Degrees')
%     grid
%     legend('show','Location','Bestoutside')
%     
%     % Elevator Down to Outputs
%     timedata = load(['./data/' files(4).name]);
%     timedata = timedata.data;
%     opt = stepDataOptions;
%     opt.StepAmplitude = -0.01;
%     for j = 1:numel(out)
%         figure(15+18+18+1+j)
%         % ROM Time response
%         [y,t] = step(sys_aug(out(j),'Elevator'),120,opt);
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(t,y,'--','DisplayName',['EAS ' num2str(eas) ' ROM']); hold on
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(timedata.t,timedata.(char(out_aswing(j)))-timedata.(char(out_aswing(j)))(1),'DisplayName',['EAS ' num2str(eas) ' Aswing']); hold on
%         title(['Elevator - 0.01 to ' out(j)])
%         xlabel('Time (s)')
%         grid
%         legend('show','Location','Bestoutside')
%     end
%         
%     % Elevator Down to Twist
%     twist_init_right = timedata.Theta2(1)-timedata.Theta4(1);
%     twist_init_left = timedata.Theta3(1)-timedata.Theta4(1);
%     figure(15+18+18+1+18+1)
%     plot(timedata.t,(timedata.Theta2-timedata.Theta4)-twist_init_right,'DisplayName',['Right Wing EAS ' num2str(eas)]); hold on
% %     plot(timedata.t,(timedata.Theta3-timedata.Theta4)-twist_init_left,'DisplayName',['Left Wing EAS ' num2str(eas)]);
%     title('Elevator - 0.01 to Twist')
%     xlabel('Time (s)')
%     ylabel('Degrees')
%     grid
%     legend('show','Location','Bestoutside')
%     
%     % Elevator Up to Outputs
%     timedata = load(['./data/' files(5).name]);
%     timedata = timedata.data;
%     opt = stepDataOptions;
%     opt.StepAmplitude = +0.01;
%     for j = 1:numel(out)
%         figure(15+18+18+18+1+j)
%         % ROM Time response
%         [y,t] = step(sys_aug(out(j),'Elevator'),120,opt);
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(t,y,'--','DisplayName',['EAS ' num2str(eas) ' ROM']); hold on
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(timedata.t,timedata.(char(out_aswing(j)))-timedata.(char(out_aswing(j)))(1),'DisplayName',['EAS ' num2str(eas) ' Aswing']); hold on
%         title(['Elevator + 0.01 to ' out(j)])
%         xlabel('Time (s)')
%         grid
%         legend('show','Location','Bestoutside')
%     end
%         
%     % Elevator Up to Twist
%     twist_init_right = timedata.Theta2(1)-timedata.Theta4(1);
%     twist_init_left = timedata.Theta3(1)-timedata.Theta4(1);
%     figure(15+18+18+1+18+18+1)
%     plot(timedata.t,(timedata.Theta2-timedata.Theta4),'DisplayName',['Right Wing EAS ' num2str(eas)]); hold on
% %     plot(timedata.t,(timedata.Theta3-timedata.Theta4),'DisplayName',['Left Wing EAS ' num2str(eas)]);
%     title('Elevator + 0.01 to Twist')
%     xlabel('Time (s)')
%     ylabel('Degrees')
%     grid
%     legend('show','Location','Bestoutside')
%     
%     % Motor Down to Outputs
%     timedata = load(['./data/' files(8).name]);
%     timedata = timedata.data;
%     opt = stepDataOptions;
%     opt.StepAmplitude = -0.01;
%     for j = 1:numel(out)
%         figure(15+18+18+18+18+1+j)
%         % ROM Time response
%         [y,t] = step(sys_aug(out(j),'Motor'),120,opt);
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(t,y,'--','DisplayName',['EAS ' num2str(eas) ' ROM']); hold on
%         ax = gca;
%         ax.ColorOrderIndex = i;
%         plot(timedata.t,timedata.(char(out_aswing(j)))-timedata.(char(out_aswing(j)))(1),'DisplayName',['EAS ' num2str(eas) ' Aswing']); hold on
%         title(['Motor - 0.01 to ' out(j)])
%         xlabel('Time (s)')
%         grid
%         legend('show','Location','Bestoutside')
%     end
%     
%     % Motor Down to Twist
%     twist_init_right = timedata.Theta2(1)-timedata.Theta4(1);
%     twist_init_left = timedata.Theta3(1)-timedata.Theta4(1);
%     figure(15+18+18+1+18+18+18+1)
%     plot(timedata.t,(timedata.Theta2-timedata.Theta4),'DisplayName',['Right Wing EAS ' eas]); hold on
% %     plot(timedata.t,(timedata.Theta3-timedata.Theta4),'DisplayName',['Left Wing EAS ' eas]);
%     title('Motor - 0.01 to Twist')
%     xlabel('Time (s)')
%     ylabel('Degrees')
%     grid
%     legend('show','Location','Bestoutside')
%     
%     % Motor Up to dihedral angle
%     timedata = timedata_motorup;
%     dihedreal_init = atan2d((timedata.RZpos2-timedata.RZpos4),(timedata.RYpos2-timedata.RYpos4));
%     dihedreal_init = dihedreal_init(1);
% 
%     timedata.dihedral_right = atan2d((timedata.RZpos2-timedata.RZpos4),abs((timedata.RYpos2-timedata.RYpos4)));
%     timedata.dihedral_left = atan2d((timedata.RZpos3-timedata.RZpos4),abs((timedata.RYpos3-timedata.RYpos4)));
%     
%     figure(15+18+18+1+18+18+18+2)
% %     plot(timedata.t,timedata.dihedral_right,'DisplayName',['Right Wing EAS ' eas]); hold on
%     plot(timedata.t,timedata.dihedral_left,'DisplayName',['Left Wing EAS ' eas]);
%     title('Motor + 0.01 to Dihedral')
%     xlabel('Time (s)')
%     ylabel('Degrees')
%     grid
%     legend('show','Location','Bestoutside')

    
    
end

% Add legends
figure(1)
title('PZ Map ROM')
legend('ROM EAS 7.5','ROM EAS 8','ROM EAS 9','ROM EAS 9.5','ROM EAS 10','ROM EAS 10.5','Location','Bestoutside')
figure(2)
title('PZ Map ASWING')
legend('ASWING EAS 7.5','ASWING EAS 8','ASWING EAS 9','ASWING EAS 9.5','ASWING EAS 10','ASWING EAS 10.5','Location','Bestoutside')
figure(3)
title('Pole Map ROM and ASWING')
legend('ROM EAS 7.5','ASWING EAS 7.5','ROM EAS 8','ASWING EAS 8','ROM EAS 9','ASWING EAS 9','ROM EAS 9.5','ASWING EAS 9.5','ROM EAS 10','ASWING EAS 10','ROM EAS 10.5','ASWING EAS 10.5','Location','Bestoutside')

prettyPlots()

% figHandles = get(groot,'Children');
% for i = 1:numel(figHandles)
%    figure(figHandles(i));
%      grid;
%    export_fig plots.pdf -append
% end