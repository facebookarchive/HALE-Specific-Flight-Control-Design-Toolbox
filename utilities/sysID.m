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
%   Develope: Abe Martin
function [ss_id] = sysID( model_name, runSysIDReport, aswing_opts )
%sysID Identifies state space model from ASWING frequency response data.
% Model is saved to a mat file entitled ss_model_name.
% case_name = '_0_10';

% Get case name from model name
% model_name = optOptions.CustomModelName;

% Open frequency response data
% fid = fopen(strcat('./CustomModels/FrequencyData/',model_name,'nonlin.bode'));
fid = fopen(strcat(model_name,'nonlin.bode'));

% Number of frequencies in data file
numFreq = aswing_opts.num_freq;

% Model input and output names
in = {'Aileron','Elevator','Rudder','Motor'};
out = {'dUx','dUy','dUz','UX','UY','UZ','Wx','Wy','Wz','RX','RY','RZ','Phi','Theta','Psi','V','beta','alpha'};

Response = [];

% Parse frequency response information from Aswing
for u = 1:numel(in)
    for y = 1:numel(out)
        % Read Aswing Nonlinear Frequency Response
        R = textscan(fid,'%f %f %f',numFreq,'CommentStyle','#','Delimiter','\t');
        R = [R{:}];
        Freq = R(:,1);
        Gain = R(:,2);
        Phase = R(:,3);
        % Convert response to complex from
        % rho e^(J theta) = rho * cos(theta) + j rho * sin(theta)
        Response(y,u,:) = Gain.*cos(deg2rad(Phase))+1i*Gain.*sin(deg2rad(Phase));
    end 
end

% Convert frequency from Hz to rad/s
Freq = Freq * (2 * pi());

% Create frequency response object.
hmore = idfrd(Response,Freq,0);
hmore.InputName = in;
hmore.OutputName = out;

% State space model estimation                                        
Options = n4sidOptions;                                              
Options.Display = 'off';                                              
% Options.EstimateCovariance = false;  
Options.EstCovar = false;
Options.N4Weight = 'CVA';                                            
Options.N4Horizon = [30 30 30];                                      

ss_id = n4sid(hmore, 30, 'Feedthrough', [true true true true], Options);

% % Save system to file
% save(strcat('./CustomModels/StateSpace/',model_name),'ss_id');

if(runSysIDReport == 1)

    % Compare poles with original system
%     fid = fopen(strcat('./CustomModels/FrequencyData/',model_name,'.e00'));
    fid = fopen(strcat(model_name,'.e00'));
    R = textscan(fid,'%f %f %f','HeaderLines',3,'Delimiter','\t');
    eigs = conj_sort(R{2},R{3},'row');
    eigs = unique(eigs);
    p_id = pole(ss_id);
    P_orig = eigs;

    figure
    pzmap(p_id,[]); hold on
    pzmap([],P_orig); hold off
    title(strcat('Pole Comparison'))
    legend('Identified','Original')

    figure
    pzmap(ss_id('Wy','Elevator')); hold on
    pzmap([],P_orig); hold off
    title(strcat('Wy Elevator Pole Comparison'))
    legend('Identified','Original')

    % Calculate natural frequencies
    [Wn_id,~,P_id] = damp(ss_id);
    Wn_orig = sqrt(real(P_orig).^2+imag(P_orig).^2);

    % Match poles between data and identified
    D = pdist2([real(P_orig) imag(P_orig)],[real(P_id) imag(P_id)]);
    P_id_copy = P_id;
    Wn_id_copy = Wn_id;
    matches = [];
    for i=1:numel(D(:,1))
        [m,j] = min(D(i,:));
        if(m<=0.05)
            matches = [matches; P_orig(i) Wn_orig(i) P_id_copy(j) Wn_id_copy(j)];
            D(:,j) = [];
            P_id_copy(j) = [];
            Wn_id_copy(j) = [];
        end
    end

    % Find non matching from both sets
    [orig_no_match,idx] = setdiff(P_orig,matches(:,1));
    orig_no_match_wn = Wn_orig(idx);
    [id_no_match,idx] = setdiff(P_id,matches(:,3));
    id_no_match_wn = Wn_id(idx);

    T1 = table(matches(:,1),matches(:,2),matches(:,3),matches(:,4),'VariableNames',{'P_orig','Wn_orig','P_id','Wn_id'})
    T2 = table(orig_no_match,orig_no_match_wn)
    T3 = table(id_no_match,id_no_match_wn)

    % Fit vs Data plots
%     figure
%     for u = 1:numel(in)
%         for y = 1:numel(out)
%             data = hmore(y,u);
%             sysID = ss_id(y,u);
%             compare(data,sysID);
%             filename = char(strcat('CustomModels/sysIDPlots/compare_', string(in(u)) ,'To', string(out(y)) ,'.png'));
%             saveas(gcf,filename)
%         end
%     end

%     % Study on best number of modes
%     x = linspace(15,100,18)
%     fit_data = []
%     for i=1:numel(x)
%         Options.N4Horizon = [x(i) x(i) x(i)];                                      
%         ss_id = n4sid(hmore, x(i), 'Feedthrough', [true true true true], Options);
%         Options.N4Horizon = 'auto';                                      
%         ss_id_auto = n4sid(hmore, x(i), 'Feedthrough', [true true true true], Options);
%         fit_data = [fit_data; x(i) ss_id.Report.Fit.MSE ss_id_auto.Report.Fit.MSE];
%     end
%     figure
%     semilogy(x,fit_data(:,2),x,fit_data(:,3))
%     legend('Horizon Match','Horizon Auto')
%     title('SysID Mean Square Error vs Order')
%     xlabel('System Order')
%     ylabel('MSE')
end

end

function vec = conj_sort(x,y,method)
%x: real part
%y: imaginary part 
len = length(x);
switch method
    case 'row' 
    vec = zeros(len*2,1);
    vec(1:2:end) = complex(x,y);
    vec(2:2:end) = conj(complex(x,y));
    
    case 'col'
    vec = zeros(len,2);    
    vec(:,1) = complex(x,y);
    vec(:,2) = conj(complex(x,y));  
end
end
