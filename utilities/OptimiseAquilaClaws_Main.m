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
%	PURPOSE:            1. Initialize the path
%                       2. Initiaze the model data
%                       3. Call the optimization function, either the one with the gain scheduling capability or then one for unique flight condition
%                           gain tuning
%                       4. Analysis of the open loop flight dynamics model with the flexible modes included to determine the best sensor/servo pair 
%                          combination - use "getAquilaModel_FltDynAnalysis" module 
%
%	INPUTS:             1. Path to be added to the existing MATLAB paths 
%                       2. Optimization configuration selector i.e., PitchLoopw/oFilts or PitchLoop etc
%                       3. set the switch to determine mutiple flight conditions are optimized or just one flight condition
%                       4. flight conditions to be optimised
%
% 
% 
% 	OUTPUTS: 
%
%
%	PRE-REQUISITES:		ASE model should be available, the standard tunbale Claws gains and coefficients should be provided
%	                    The optimization configuration function (OptConfigSetup) should be prepared to setup the optimisation
%                       objects, variables and loops 					
%
%	LIMITATIONS:		Currently many
%
%
%	DEVELOPER :			H. Bolandhemmat, Abe Martin, David Flamholz (Facebook Connectivity Lab)
%
%

% Initialization parameters/variables files will be added here:

OptimiseAquilaClaws_init;

% 
% 
% set the following switch to determine if the optimizer is called to
% adjust the gains over various flight conditions or just one specific flight condition! changing the
% switch will change the simulink model called.

% OptConfig, Optimize4MultipleFlts and ReadFltCndFrmCSVFile are the only Optimisation configuration options that are defined in the top-level script.

Options.Optimize4MultipleFlts = FALSE ;
Options.ReadFltCndFrmCSVFile  = FALSE ;    %if TRUE, make sure that the input flight conditions for which the CLaws are to be designed are inputted in FltCndInput.csv

% Set the Optimisation Options:
if( Options.Optimize4MultipleFlts == TRUE )

%     To define the OptConfig add the following char to the OptConfig:

%      1. 'PitchLoop':                    for pitch loop design,
%      2. 'ClimbRateLoop':                for ClimbRate loop design 
%      3. 'TASLoop':                      for TAS loop design
%      4. 'PwrLoop':                      for pwr loop design      
%      5. 'AltLoop':                      for altitude loop design 
%      6. 'RollLoop':                     for Roll/rate loop design 
%      7. 'GndTrkLoop':                   for Ground track loop design 
%      8. 'YawRateLoop':                  for Yaw rate loop design 
%      9. 'FlexLoop':                     for Flexible mode(s) control loop design 
%      9. 'GLALoop':                      for GLA control loop design 

    Options.OptConfig = 'PitchLoop/ClimbRateLoop/TASLoop/PwrLoop/AltLoop/RollLoop/GndTrkLoop/YawRateLoop';     %'PitchLoop/ClimbRateLoop/TASLoop/PwrLoop/AltLoop';
    
    if ( Options.ReadFltCndFrmCSVFile == TRUE )
       [flt.alts, flt.eass, flt.gammas, flt.phis]=getFltCndInput('FltCndInput.csv');
    else
        flt.alts        = [0 1500 3000 4500 6000 7500]; % 9000 10500 12000] ;
        flt.eass        = [9.93 10.97 12.44] ; % 9.93
        flt.gammas      = [ 0 ] ;
        flt.phis        = [ 0 ] ;
    end
    
    %Initialization of the Claws parameters,
    AquilaDiscreteClawsParameters_GS;

else
    Options.OptConfig = 'PitchLoop/ClimbRateLoop/TASLoop/PwrLoop/AltLoop';
%     /RollLoop/GndTrkLoop/YawRateLoop
    
    if ( Options.ReadFltCndFrmCSVFile == TRUE )
       [flt.alts, flt.eass, flt.gammas, flt.phis]=getFltCndInput('FltCndInput.csv');
    else

        % Determine the flight conditions over which the control system is designed
        flt.alts   = [ 0 ] ;
        flt.eass   = [ 8 ] ;   %9.93
        flt.gammas = [ 0 ] ;
        flt.phis   = [ 0 ] ;
    end
    
    %Initialization of the Claws parameters,
    AquilaDiscreteClawsParameters;
    
end

% Call the Optimisation routine

if( Options.Optimize4MultipleFlts == TRUE )
    [OptOut] = OptimizeAquilaClaws_Discrete_GS('LinLongNormal_Aquila_ClawsGainSchedule_Discrete',flt,'OptConfigSetup',Options);
else
    [OptOut] = OptimizeAquilaClaws_Discrete_GS('LinLongNormal_Aquila_Claws_Discrete',flt,'OptConfigSetup',Options);
end

