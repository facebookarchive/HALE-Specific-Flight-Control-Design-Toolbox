function [] = ASWING2MATLAB (model_name,aswing_opts,debugROM,useSysID,runSysIDReport)
%% This function calls Aswing to generate a reduced order aircraft model
% and parses it into a Matlab sys, or estimates a model using Aswing bode
% data

% Run aswing to generate ROM and bode data
run_aswing(aswing_opts)

% IO labels
in = {'Aileron','Elevator','Rudder','Motor'};
out = {'dUx','dUy','dUz','UX','UY','UZ','Wx','Wy','Wz','RX','RY','RZ','Phi','Theta','Psi','V','beta','alpha'};

if(useSysID==true)
    % Get system from system ID using aswing bode info
    sys = sysID('AQ3', runSysIDReport, aswing_opts);
    
else
%% Load Aswing ROM
sys = ParseROM (numel(in),numel(out),'AQ3');
sys.InputName = in;
sys.OutputName = out;
sys.Name = 'Matlab ROM';

% Load twist ROM
sys_twist = ParseROM (numel(in),4,'AQ3twist');
sys_twist.InputName = in;
sys_twist.OutputName = {'Theta1','Theta2','Theta3','Theta4'};
sys_twist.Name = 'Twist ROM';
end

% Save system
save(strcat('../CustomModels/StateSpace/',model_name),'sys');

if(useSysID~=true)
    % Save twist
    save(strcat('../CustomModels/StateSpace/',model_name,'_twist'),'sys_twist');
end

if(debugROM==true)
    %% Additional code for plotting and debugging the transferred model.
    
    % Open nonlinear bode file
    fid2 = fopen('AQ3nonlin.bode');

    Response2 = [];

    % Parse Bode information from Aswing
    for u = 1:numel(in)
        for y = 1:numel(out)
            % Read Aswing Nonlinear Frequency Response
            R = textscan(fid2,'%f %f %f',aswing_opts.num_freq,'CommentStyle','#','Delimiter','\t');
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
    
    if(useSysID~=true)
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
    C_new(15,end) = -1; % Add new psi output

    D_new([10:12,15],:) = 0; % Clear old passthroughs 

    sys_aug = ss(A_new,B_new,C_new,D_new);
    sys_aug.InputName = in;
    sys_aug.OutputName = out;
    sys_aug.Name = 'Augmented ROM';
    
    else
        sys_aug = sys;
    end 
    
    % Compare sys to data
    [~,fit] = compare(fr_data_nonlin,sys_aug);
    
    % List bad IO frequency fits
    [o,i] = ind2sub(size(fit),find(fit<0));
    disp('Potential Bad Channels')
    for k = 1:numel(o)
       disp(strcat(in(i(k)),'_to_',out(o(k)))) 
       figure
       compare(fr_data_nonlin(o(k),i(k)),sys_aug(o(k),i(k)),'--r');
       legend('Location','best')
       grid
       figure
       step(sys_aug(o(k),i(k)),60)
       grid
    end
    
    % Always plot these important channels whether they are good or bad
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
    for i=1:numel(pairs)
       figure
       k = pairs{i}{2};
       j = pairs{i}{1};
       compare(fr_data_nonlin(k,j),sys_aug(k,j),'--r');
       legend('Location','best')
       grid
    end
    
    % Display mean NRMSE fit
    mean_fit = mean(fit(:));
    disp(mean_fit)
    
    % Compare poles with original system
    fid = fopen(strcat('AQ3.e00'));
    R = textscan(fid,'%f %f %f','HeaderLines',3,'Delimiter','\t');
    eigs = conj_sort(R{2},R{3},'row');
    eigs = unique(eigs);
    p_ROM = pole(sys_aug);
    P_orig = eigs;
    figure
    pzmap(p_ROM,[]); hold on
    pzmap([],P_orig); hold off
    title(strcat('Pole Comparison Alt: ', num2str(aswing_opts.altitude), ' EAS: ', num2str(aswing_opts.eas)))
    legend('ROM','Original')
    
    prettyPlots();
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