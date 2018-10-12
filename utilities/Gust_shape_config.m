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

% gust simulations function

%Initialize the gust parameters
lamA = 0.1;   % Pulse amplification factor
lamC = 1;   % Dropoff factor

% Gust initial magnitude
init_gust = 0;


%Timing parameters
tstart = 5;             % gust start time in sec
tend   = 10;            % simulation end time in sec
duration  = 30 ;        %gust duration time in sec

time_step = 1/25 ;   %in sec

%Initialize gust vectors
Wgust_pulse_exponential = init_gust*ones(1,duration/time_step+1) ;
Wgust_pulse_harmonic    = init_gust*ones(1,duration/time_step+1) ;
time                    = zeros(1,duration/time_step+1) ;
i = 1;

for tt=0:time_step:tend
    
    
    if( tt>=tstart && tt<=tend)
        % Compute gaussian pulse exponential and derivative factor
        gaussexp = -(tt - tstart)^2 / (2*lamC^2);

        % Update input based on time
        Wgust_pulse_exponential(i+1)     = Wgust_pulse_exponential(i)     + lamA * exp(gaussexp);
    end
    
    % Compute Simple harmonic pulse
    u_gust  = 0.5; % Gust magnitude [m/s]
    f_gust  = 1.0; % Gust frequency [Hz]

    % Multiplier for setting input signal frequency the same as flutter frequency
    lamB = 2 * pi * f_gust;

    % Update input based on time, for only one gust period
    if tt > tstart && tt < (tstart + 1/f_gust)
        Wgust_pulse_harmonic(i+1)     = u_gust * (1 - cos(lamB * (tt - tstart)));
    end

    time(i) = tt ;
    
    i = i+1 ;
end

figure;plot(time,Wgust_pulse_exponential);xlabel('time (sec)');ylabel('gust magnitude (m/s)');grid;title('Exponential Gust Pulse'); 
figure;plot(time,Wgust_pulse_harmonic);xlabel('time (sec)');ylabel('gust magnitude (m/s)');grid;title('Harmonic Gust Pulse');