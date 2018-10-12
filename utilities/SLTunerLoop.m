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
%   This Loop Class is a general parent Loop class. Given a LoopStruct
%   (defined in the _LoopOptObjectives files), as well as Options and an
%   sltuner_interface such as ST1 in the GS file, we have associated arrays 
%   of transfer functions associated with each Control Loop 
%   reason for making this general class is that CL and L output from the
%   (PitchLoop, YawRateLoop, etc). Specific elements of these Arrays in the 
%   parent SLTunerLoop class are later fed into a SubLoop which then 
%   represents a transfer function for CL and L.

%   Inputs: A LoopStruct associated with either PitchLoop, ClimbRateLoop,
%   etc., the ST1 sltuner_interface and Options. The LoopStruct will be
%   updated before being input into this class to contain CLgroup, Lgroup,
%   etc (arrays of transfer function). 
%   Outputs: A set of methods called with a given set of indices to get a 
%   single CL (close loop transfer function) or a single L (Loop gain), etc
%   associated with a single set of flight conditions.

classdef SLTunerLoop<handle
    properties
        LoopStruct
        % LoopStruct is, among other things, an abstraction for all the
        % information contained in a given Loop's ViewSpec plot
        sltuner_interface
        Options
    end
    
    methods
        % Constructor
        function obj=SLTunerLoop(LoopStruct,sltuner_interface,Options)
            obj.LoopStruct=LoopStruct;
            obj.sltuner_interface=sltuner_interface;
            obj.Options=Options;
        end
        
        
        % Here we begin defining methods for Adding properties to a
        % SubLoopStruct that can be used to initialize a SubLoop object
        % associated with specific indices
        
        
        % This First method adds this class's LoopStruct propert to a
        % specifc SubLoopStruct that will be used to create a SubLoop
        function [SubLoopStruct]=AddLoopStruct(obj,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            SubLoopStruct.LoopStruct=obj.LoopStruct;
        end
        
        function [SubLoopStruct]=AddOptions(obj,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            SubLoopStruct.Options=obj.Options;
        end
        
        % This method adds a closed loop transfer function associated
        % with specific indices to SubLoopStruct
        function [SubLoopStruct]=AddCL(obj,indices,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            
            if obj.Options.Optimize4MultipleFlts
                SubLoopStruct.CL=...
                   obj.LoopStruct.CLgroup(indices.gamma_index,...
                                          indices.phi_index,...
                                          indices.alt_index,...
                                          indices.eas_index);
            else
                SubLoopStruct.CL=obj.LoopStruct.CLgroup;
            end
        end
        
        % This method adds a Loop Gain transfer function associated
        % with specific indices to SubLoopStruct
        function [SubLoopStruct]=AddL(obj,indices,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            
            if obj.Options.Optimize4MultipleFlts
                SubLoopStruct.L=...
                    obj.LoopStruct.Lgroup(indices.gamma_index,...
                                          indices.phi_index,...
                                          indices.alt_index,...
                                          indices.eas_index); 
            else
                SubLoopStruct.L=obj.LoopStruct.Lgroup;
            end
        end
        
        
        % This method adds a pair of Input and Output margins transfer 
        % functions associated with specific indices to SubLoopStruct
        function [SubLoopStruct]=AddMarginsTFs(obj,indices,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            
            if obj.Options.Optimize4MultipleFlts
                SubLoopStruct.MarginsInputTF=...
                obj.LoopStruct.MarginsInputTFgroup(indices.gamma_index,...
                                                   indices.phi_index,...
                                                   indices.alt_index,...
                                                   indices.eas_index);
                                                
                SubLoopStruct.MarginsOutputTF=...
                obj.LoopStruct.MarginsOutputTFgroup(indices.gamma_index,...
                                                    indices.phi_index,...
                                                    indices.alt_index,...
                                                    indices.eas_index);
            else
                SubLoopStruct.MarginsInputTF=obj.LoopStruct.MarginsInputTFgroup;
                SubLoopStruct.MarginsOutputTF=obj.LoopStruct.MarginsOutputTFgroup;
            end
        end
        
        
        % This method adds a ViewSpecSubStruct associated
        % with specific indices to SubLoopStruct
        function [SubLoopStruct]=AddViewSpecSubStruct(obj,indices,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            % Might have to add condition here for single vs. multiple
            % flight conditions
            VS=obj.LoopStruct.ViewSpecStruct;
            ViewSpecSubStruct=struct;
            % the following set of if statements is to check which
            % ViewSpec Plots are included and make those plots' associated
            % data structures properties of the ViewSpecSubStruct. Some of
            % the names should be self-evident, indicating which plots they
            % would come from. S_array is an array containing the
            % Sensitivity Line structures each containing the Data to get
            % its respective Sensitivity Transfer functions associated with
            % the Singular Values Plot when 'Freq_Shaping' is a tuning
            % goal. T_array is the complementary sensitivity
            if isfield(VS,'StepResponse_array')
                ViewSpecSubStruct.StepResponseData=...
                              VS.StepResponse_array(indices.alt_index,...
                                                    indices.eas_index,...
                                                    indices.gamma_index,...
                                                    indices.phi_index);
            end
            
            if isfield(VS,'LoopGains_array')
                ViewSpecSubStruct.LoopGainData=...
                                 VS.LoopGains_array(indices.alt_index,...
                                                    indices.eas_index,...
                                                    indices.gamma_index,...
                                                    indices.phi_index);
            end
            
            if isfield(VS,'T_array')
                ViewSpecSubStruct.TData=...
                                         VS.T_array(indices.alt_index,...
                                                    indices.eas_index,...
                                                    indices.gamma_index,...
                                                    indices.phi_index);
            end
            
            if isfield(VS,'S_array')
                ViewSpecSubStruct.SData=...
                                         VS.S_array(indices.alt_index,...
                                                    indices.eas_index,...
                                                    indices.gamma_index,...
                                                    indices.phi_index);
            end
            SubLoopStruct.ViewSpecSubStruct=ViewSpecSubStruct;
        end
           
        %%    
        %% Add Method Here to Add more info into a SubLoopStruct which can 
        %% be passed to a SubLoop enabling more metrics to be calculated 
        %% and presented
        %%
        
        function [SubLoopStruct]=AddSensitivityTF(obj,indices,OldSubLoopStruct)
            SubLoopStruct=OldSubLoopStruct;
            if isfield(obj.LoopStruct,'SensitivityTFgroup')
                if obj.Options.Optimize4MultipleFlts
                    SubLoopStruct.SensitivityTF=...
                      obj.LoopStruct.SensitivityTFgroup(indices.gamma_index,...
                                                        indices.phi_index,...
                                                        indices.alt_index,...
                                                        indices.eas_index);
                else
                    SubLoopStruct.SensitivityTF=...
                                             obj.LoopStruct.SensitivityTFgroup;
                end
            end
        end
        
        
        % Create SubLoopStruct that can be used to define a SubLoop object.
        % This is where we would add an Add_ function if we wanted to add
        % new info to SubLoop
        function [SubLoopStruct]=getSubLoopStruct(obj,indices)
            SubLoopStruct=struct;
            SubLoopStruct=obj.AddLoopStruct(SubLoopStruct);
            SubLoopStruct=obj.AddOptions(SubLoopStruct);
            SubLoopStruct=obj.AddCL(indices,SubLoopStruct);
            SubLoopStruct=obj.AddL(indices,SubLoopStruct);
            SubLoopStruct=obj.AddMarginsTFs(indices,SubLoopStruct);
            SubLoopStruct=obj.AddViewSpecSubStruct(indices,SubLoopStruct);
            %% Call new Add_ methods here to add to SubLoopStruct
            %%
            SubLoopStruct=obj.AddSensitivityTF(indices,SubLoopStruct);
        end

    end
end
       