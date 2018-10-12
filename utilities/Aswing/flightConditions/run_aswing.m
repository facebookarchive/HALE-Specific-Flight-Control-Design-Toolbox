function run_aswing( aswing_opts )
%run_aswing runs aswing to output both a reduced order model and bode data
%at the desired conditions.  The outputs are saved to AF.bode, AF.e00, and
%AF.rom

%% ROM
% Read ROM input file
fid = fopen('ComROM_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'num_modes',num2str(aswing_opts.num_modes));

% Write new input file for aswing
fid = fopen('ComASE_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComASE_.txt');

%% Bode
% Read Bode input file
fid = fopen('ComBode_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'num_modes',num2str(aswing_opts.num_modes));
f = strrep(f,'freq_min',num2str(aswing_opts.freq_min));
f = strrep(f,'freq_max',num2str(aswing_opts.freq_max));
f = strrep(f,'num_freq',num2str(aswing_opts.num_freq));

% Write new input file for aswing
fid = fopen('ComBode_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComBode_.txt');

%% Time Steady
% Use modified aswing model with additional sensors on wingtips
aswing_opts.asw_file_name = 'AQ3mod.asw';
% Read Time input file
fid = fopen('ComTime_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

%% Parse trim to get input values
trim = parseTrim('trimstate.dat',4,4);
Peng_trim = str2double(trim.Peng1);
Flap1_trim = str2double(trim.Flap1);
Flap2_trim = str2double(trim.Flap2);
Flap3_trim = str2double(trim.Flap3);

%% Step Tests
% Motor Up
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim+0.01));
f = strrep(f,'flap1_value',num2str(Flap1_trim));
f = strrep(f,'flap2_value',num2str(Flap2_trim));
f = strrep(f,'flap3_value',num2str(Flap3_trim));
f = strrep(f,'timefilename1','AQ3motorup1.dat');
f = strrep(f,'timefilename2','AQ3motorup2.dat');
f = strrep(f,'timefilename3','AQ3motorup3.dat');
f = strrep(f,'timefilename4','AQ3motorup4.dat');
f = strrep(f,'timefilename5','AQ3motorup5.dat');
f = strrep(f,'timefilename6','AQ3motorup6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Motor Down
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim-0.01));
f = strrep(f,'flap1_value',num2str(Flap1_trim));
f = strrep(f,'flap2_value',num2str(Flap2_trim));
f = strrep(f,'flap3_value',num2str(Flap3_trim));
f = strrep(f,'timefilename1','AQ3motordown1.dat');
f = strrep(f,'timefilename2','AQ3motordown2.dat');
f = strrep(f,'timefilename3','AQ3motordown3.dat');
f = strrep(f,'timefilename4','AQ3motordown4.dat');
f = strrep(f,'timefilename5','AQ3motordown5.dat');
f = strrep(f,'timefilename6','AQ3motordown6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Flap 1 Up
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim));
f = strrep(f,'flap1_value',num2str(Flap1_trim+0.01));
f = strrep(f,'flap2_value',num2str(Flap2_trim));
f = strrep(f,'flap3_value',num2str(Flap3_trim));
f = strrep(f,'timefilename1','AQ3flap1up1.dat');
f = strrep(f,'timefilename2','AQ3flap1up2.dat');
f = strrep(f,'timefilename3','AQ3flap1up3.dat');
f = strrep(f,'timefilename4','AQ3flap1up4.dat');
f = strrep(f,'timefilename5','AQ3flap1up5.dat');
f = strrep(f,'timefilename6','AQ3flap1up6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Flap 1 Down
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim));
f = strrep(f,'flap1_value',num2str(Flap1_trim-0.01));
f = strrep(f,'flap2_value',num2str(Flap2_trim));
f = strrep(f,'flap3_value',num2str(Flap3_trim));
f = strrep(f,'timefilename1','AQ3flap1down1.dat');
f = strrep(f,'timefilename2','AQ3flap1down2.dat');
f = strrep(f,'timefilename3','AQ3flap1down3.dat');
f = strrep(f,'timefilename4','AQ3flap1down4.dat');
f = strrep(f,'timefilename5','AQ3flap1down5.dat');
f = strrep(f,'timefilename6','AQ3flap1down6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Flap 2 Up
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim));
f = strrep(f,'flap1_value',num2str(Flap1_trim));
f = strrep(f,'flap2_value',num2str(Flap2_trim+0.01));
f = strrep(f,'flap3_value',num2str(Flap3_trim));
f = strrep(f,'timefilename1','AQ3flap2up1.dat');
f = strrep(f,'timefilename2','AQ3flap2up2.dat');
f = strrep(f,'timefilename3','AQ3flap2up3.dat');
f = strrep(f,'timefilename4','AQ3flap2up4.dat');
f = strrep(f,'timefilename5','AQ3flap2up5.dat');
f = strrep(f,'timefilename6','AQ3flap2up6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Flap 2 Down
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim));
f = strrep(f,'flap1_value',num2str(Flap1_trim));
f = strrep(f,'flap2_value',num2str(Flap2_trim-0.01));
f = strrep(f,'flap3_value',num2str(Flap3_trim));
f = strrep(f,'timefilename1','AQ3flap2down1.dat');
f = strrep(f,'timefilename2','AQ3flap2down2.dat');
f = strrep(f,'timefilename3','AQ3flap2down3.dat');
f = strrep(f,'timefilename4','AQ3flap2down4.dat');
f = strrep(f,'timefilename5','AQ3flap2down5.dat');
f = strrep(f,'timefilename6','AQ3flap2down6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Flap 3 Up
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim));
f = strrep(f,'flap1_value',num2str(Flap1_trim));
f = strrep(f,'flap2_value',num2str(Flap2_trim));
f = strrep(f,'flap3_value',num2str(Flap3_trim+0.01));
f = strrep(f,'timefilename1','AQ3flap3up1.dat');
f = strrep(f,'timefilename2','AQ3flap3up2.dat');
f = strrep(f,'timefilename3','AQ3flap3up3.dat');
f = strrep(f,'timefilename4','AQ3flap3up4.dat');
f = strrep(f,'timefilename5','AQ3flap3up5.dat');
f = strrep(f,'timefilename6','AQ3flap3up6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');

% Flap 3 Down
% Read Time input file
fid = fopen('ComStep_template.txt','r');
f = fread(fid);

% Replace desired values in aswing input file
f = strrep(f,'asw_file_name',aswing_opts.asw_file_name);
f = strrep(f,'point_file_name',aswing_opts.point_file_name);
f = strrep(f,'eas',num2str(aswing_opts.eas));
f = strrep(f,'altitude',num2str(aswing_opts.altitude));
f = strrep(f,'peng_value',num2str(Peng_trim));
f = strrep(f,'flap1_value',num2str(Flap1_trim));
f = strrep(f,'flap2_value',num2str(Flap2_trim));
f = strrep(f,'flap3_value',num2str(Flap3_trim-0.01));
f = strrep(f,'timefilename1','AQ3flap3down1.dat');
f = strrep(f,'timefilename2','AQ3flap3down2.dat');
f = strrep(f,'timefilename3','AQ3flap3down3.dat');
f = strrep(f,'timefilename4','AQ3flap3down4.dat');
f = strrep(f,'timefilename5','AQ3flap3down5.dat');
f = strrep(f,'timefilename6','AQ3flap3down6.dat');

% Write new input file for aswing
fid = fopen('ComTime_.txt','w');
fprintf(fid,'%s',f);
fclose(fid);

% Execute aswing commands
system('./aswing<ComTime_.txt');
end

