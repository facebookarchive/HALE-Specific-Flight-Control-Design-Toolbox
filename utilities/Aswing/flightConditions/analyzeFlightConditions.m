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
easList = [7.5 8.5 9.5 10.5];

for i=1:numel(easList)
    %% Run Aswing to Collect data
    aswing_opts.asw_file_name = 'AQ3.asw';
    aswing_opts.point_file_name = 'SLF.pnt';
    aswing_opts.eas = easList(i);
    aswing_opts.altitude = 0;
    aswing_opts.num_modes = 60;
    aswing_opts.freq_min = 0.001;
    aswing_opts.freq_max = 1.6;
    aswing_opts.num_freq = 120;

    run_aswing(aswing_opts);

    %% Parse Data

    in = {'Aileron','Elevator','Rudder','Motor'};
    out = {'dUx','dUy','dUz','UX','UY','UZ','Wx','Wy','Wz','RX','RY','RZ','Phi','Theta','Psi','V','beta','alpha'};

    % Load Aswing ROM
    sys = ParseROM (numel(in),numel(out),'AQ3',aswing_opts.eas);
    sys = balred(sys,50);
    sys.InputName = in;
    sys.OutputName = out;
    sys.Name = strcat('Matlab_ROM_EAS_',num2str(aswing_opts.eas));

    % Save ROM
    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_sys.mat'),'sys');

    % Open bode file
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
    fr_data_nonlin.Name = strcat('ASWING_Nonlinear_EAS_',num2str(aswing_opts.eas));

    % Save ASWING Bode Data
    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_nonlinBode.mat'),'fr_data_nonlin');

    % Load Trim Time Data
    trim_data1 = parseTimeData('AQ3_timedata1.dat',4,203);
    trim_data2 = parseTimeData('AQ3_timedata2.dat',4,203);
    trim_data3 = parseTimeData('AQ3_timedata3.dat',4,203);
    trim_data4 = parseTimeData('AQ3_timedata4.dat',4,203);
    trim_data5 = parseTimeData('AQ3_timedata5.dat',4,203);
    trim_data6 = parseTimeData('AQ3_timedata6.dat',4,203);
    data = [trim_data1;trim_data2;trim_data3;trim_data4;trim_data5;trim_data6];

    % Save Time Data
    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_trim.mat'),'data');

    % Load Motor Up
    data1 = parseTimeData('AQ3motorup1.dat',4,203);
    data2 = parseTimeData('AQ3motorup2.dat',4,203);
    data3 = parseTimeData('AQ3motorup3.dat',4,203);
    data4 = parseTimeData('AQ3motorup4.dat',4,203);
    data5 = parseTimeData('AQ3motorup5.dat',4,203);
    data6 = parseTimeData('AQ3motorup6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_motorup.mat'),'data');

    % Load Motor Down
    data1 = parseTimeData('AQ3motordown1.dat',4,203);
    data2 = parseTimeData('AQ3motordown2.dat',4,203);
    data3 = parseTimeData('AQ3motordown3.dat',4,203);
    data4 = parseTimeData('AQ3motordown4.dat',4,203);
    data5 = parseTimeData('AQ3motordown5.dat',4,203);
    data6 = parseTimeData('AQ3motordown6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_motordown.mat'),'data');

    % Load Flap 1 Up
    data1 = parseTimeData('AQ3flap1up1.dat',4,203);
    data2 = parseTimeData('AQ3flap1up2.dat',4,203);
    data3 = parseTimeData('AQ3flap1up3.dat',4,203);
    data4 = parseTimeData('AQ3flap1up4.dat',4,203);
    data5 = parseTimeData('AQ3flap1up5.dat',4,203);
    data6 = parseTimeData('AQ3flap1up6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_flap1up.mat'),'data');

    % Load Flap 1 Down
    data1 = parseTimeData('AQ3flap1down1.dat',4,203);
    data2 = parseTimeData('AQ3flap1down2.dat',4,203);
    data3 = parseTimeData('AQ3flap1down3.dat',4,203);
    data4 = parseTimeData('AQ3flap1down4.dat',4,203);
    data5 = parseTimeData('AQ3flap1down5.dat',4,203);
    data6 = parseTimeData('AQ3flap1down6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_flap1down.mat'),'data');

    % Load Flap 2 Up
    data1 = parseTimeData('AQ3flap2up1.dat',4,203);
    data2 = parseTimeData('AQ3flap2up2.dat',4,203);
    data3 = parseTimeData('AQ3flap2up3.dat',4,203);
    data4 = parseTimeData('AQ3flap2up4.dat',4,203);
    data5 = parseTimeData('AQ3flap2up5.dat',4,203);
    data6 = parseTimeData('AQ3flap2up6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_flap2up.mat'),'data');

    % Load Flap 2 Down
    data1 = parseTimeData('AQ3flap2down1.dat',4,203);
    data2 = parseTimeData('AQ3flap2down2.dat',4,203);
    data3 = parseTimeData('AQ3flap2down3.dat',4,203);
    data4 = parseTimeData('AQ3flap2down4.dat',4,203);
    data5 = parseTimeData('AQ3flap2down5.dat',4,203);
    data6 = parseTimeData('AQ3flap2down6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_flap2down.mat'),'data');

    % Load Flap 3 Up
    data1 = parseTimeData('AQ3flap3up1.dat',4,203);
    data2 = parseTimeData('AQ3flap3up2.dat',4,203);
    data3 = parseTimeData('AQ3flap3up3.dat',4,203);
    data4 = parseTimeData('AQ3flap3up4.dat',4,203);
    data5 = parseTimeData('AQ3flap3up5.dat',4,203);
    data6 = parseTimeData('AQ3flap3up6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_flap3up.mat'),'data');

    % Load Flap 3 Down
    data1 = parseTimeData('AQ3flap3down1.dat',4,203);
    data2 = parseTimeData('AQ3flap3down2.dat',4,203);
    data3 = parseTimeData('AQ3flap3down3.dat',4,203);
    data4 = parseTimeData('AQ3flap3down4.dat',4,203);
    data5 = parseTimeData('AQ3flap3down5.dat',4,203);
    data6 = parseTimeData('AQ3flap3down6.dat',4,203);
    data = [data1;data2;data3;data4;data5;data6];

    save(strcat('./data/EAS',num2str(aswing_opts.eas),'_flap3down.mat'),'data');
    
    %% Remove temp files
    delete *.dat
end