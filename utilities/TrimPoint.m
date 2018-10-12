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
classdef TrimPoint<handle
    properties
        % TrimPoint_struct contains a single flight condtion, 
        % sltuner interface, Options, and indices
        TrimPointStruct
        Options
        
    end
    
    
    methods
        % Constructor
        function obj=TrimPoint(TrimPointStruct,Options)
            obj.TrimPointStruct=TrimPointStruct;
            obj.Options=Options;
        end
        
        function [FlightConditionTable]=getFlightConditionTable(obj)
            alts=obj.TrimPointStruct.FlightCondtion.alts;
            eass=obj.TrimPointStruct.FlightCondtion.eass;
            gammas=obj.TrimPointStruct.FlightCondtion.gammas;
            phis=obj.TrimPointStruct.FlightCondtion.gammas;
            FlightConditionTable=table(alts,eass,gammas,phis);
        end
        
        
        function obj=AddSLTunerLoops(obj,SLTunerLoop)
            if ~isfield(obj.TrimPointStruct,'SLTunerLoops')
                obj.TrimPointStruct=setfield(obj.TrimPointStruct,...
                                                    'SLTunerLoops',struct);
            end
            Loop_name=SLTunerLoop.LoopStruct.Name;
            CL=SLTunerLoop.getSingleCL(obj.TrimPointStruct.indices);
            L=SLTunerLoop.getSingleL(obj.TrimPointStruct.indices);
            
            subLoop=SubLoop(CL,L);
            
            obj.TrimPointStruct.SLTunerLoops=...
                              setfield(obj.TrimPointStruct.SLTunerLoops,...
                                       Loop_name,subLoop);
        end
        
        
        function obj=AddCoeffs(obj,sltuner_interface)
            BlockParameters=sltuner_interface.getBlockParam;
            fields=fieldnames(BlockParameters);
            n_fields=length(fields);
            Coeffs=struct;
            if obj.Options.Optimize4MultipleFlts
                % Get sltuner interface parameters and fieldnames to jump
                % into subfields. We will iterate over these fieldnames to
                % get the associated values and names
                BlockVals=sltuner_interface.getBlockValue;
                for i_n=1:n_fields
                    name_field_i=getfield(BlockParameters,...
                                          char(fields(i_n)));
                    ppty_name=name_field_i.Name;
                    final_field_i=getfield(BlockVals,char(fields(i_n)));
                    val=...
                        final_field_i(...
                        obj.TrimPointStruct.indices.gamma_index,...
                        obj.TrimPointStruct.indices.phi_index,...
                        obj.TrimPointStruct.indices.alt_index,...
                        obj.TrimPointStruct.indices.eas_index);
                    Coeffs=setfield(Coeffs,ppty_name,val);
                end
                obj.TrimPointStruct=setfield(obj.TrimPointStruct,...
                                             'Coeffs',Coeffs);
                obj.TrimPointStruct.Coeffs=...
                                   orderfields(obj.TrimPointStruct.Coeffs);
            else
                for i_n=1:n_fields
                    final_field_i=...
                                getfield(BlockParameters,char(fields(i_n)));
                    ppty_name=final_field_i.Name;
                    val=final_field_i.Value;
                    Coeffs=setfield(Coeffs,ppty_name,val);
                end
                obj.TrimPointStruct=setfield(obj.TrimPointStruct,...
                                             'Coeffs',Coeffs);
                obj.TrimPointStruct.Coeffs=...
                                   orderfields(obj.TrimPointStruct.Coeffs);
            end
        end
        
        
        function [CoeffsTable]=getCoeffsTable(obj,sltuner_interface)
            if nargin<2
                CoeffsTable=struct2table(obj.TrimPointStruct.Coeffs);
            else
                if isfield(obj.TrimPointStruct,'Coeffs')
                    CoeffsTable=struct2table(obj.TrimPointStruct.Coeffs);
                else
                    obj=obj.AddCoeffs(sltuner_interface);
                    CoeffsTable=struct2table(obj.TrimPointStruct.Coeffs);
                end
            end
        end
        
        % Given a specific SLTunerLoop object, this function assumes the
        % appropriate Loop has been added as a subfield to
        % TrimPointStruct.SLTunerLoops using the AddSLTunerLoop method and 
        % then gets a cell array of characteristics 
        % (Overshoot, Risetime, etc) and an associated mapping to names 
        % that will go in a table with new_title_row 
        % ('Pitch_Loop_Overshoot','Pitch_Loop_RiseTime',etc).
        % These two outputs can be used to create a table in later
        % functions
        function [Char_array,new_title_row]=getLoopChars(obj,SLTunerLoop)
            subLoop=getfield(obj.TrimPointStruct.SLTunerLoops,...
                                  SLTunerLoop.LoopStruct.Name);
            Loop_titles=SLTunerLoop.LoopStruct.title_row;
            Step_info=subLoop.StepInfo;
            field_names=fieldnames(Step_info);
            new_title_row={};
            title_row_index=0;
            Char_array={};
            for i_field=1:length(field_names)
                % See if any of the fields of Step_info are included 
                % in the associated LoopStructs title_row
                container_row=contains(Loop_titles,...
                                       field_names(i_field));
                % Checking if there is a 1 in containter row (meaning
                % we have the field in question in title row and
                % therefore want to include it). Then getting
                % associated index of title_row to create array of
                % characteristics with correct mappings to title_row
                if ismember(1,container_row)
                    title_row_index=title_row_index+1;
                    %char_index=find(container_row==1);
                    new_title_row{title_row_index}=...
                                                Loop_titles{container_row};
                    Char_array{title_row_index}=getfield(Step_info,...
                                               char(field_names(i_field)));
                end
            end
            % Check if we want to add other characteristics such as
            % crossover frequency, bandwidth, or margins
            Crossover_container_row=contains(Loop_titles,'Crossover');
            if ismember(1,Crossover_container_row)
                %Crossover_index=find(Crossover_container_row==1);
                new_title_row{length(new_title_row)+1}=...
                                      Loop_titles{Crossover_container_row};
                if obj.Options.getAllCrossovers
                    Char_array{length(Char_array)+1}={subLoop.Crossover};
                else
                    Char_array{length(Char_array)+1}=...
                                                    subLoop.Last_crossover;
                end

            end
            
            GainMargin_container_row=contains(Loop_titles,'GainMargin');
            if ismember(1,GainMargin_container_row)
                new_title_row{length(new_title_row)+1}=...
                                     Loop_titles{GainMargin_container_row};
                [Gm_dB,Pm,Wgm,Wpm]=subLoop.getMargins;
                Char_array{length(Char_array)+1}=Gm_dB;
            end
            
            PhaseMargin_container_row=contains(Loop_titles,'PhaseMargin');
            if ismember(1,PhaseMargin_container_row)
                new_title_row{length(new_title_row)+1}=...
                                    Loop_titles{PhaseMargin_container_row};
                [Gm_dB,Pm,Wgm,Wpm]=subLoop.getMargins;
                Char_array{length(Char_array)+1}=Pm;
            end
        end
                
                
        % Creates a table of characteristics associated with a single loop
        % e.g. obj.TrimPointStruct.SLTunerLoops.PitchLoop. This table can be
        % combined with the tables associated with other loops as well as a
        % coeffs table to get a combined table associated with each trim
        % point
        function [SingleLoopTable]=getLoopTable(obj,SLTunerLoop)
            if ~isfield(obj.TrimPointStruct,'SLTunerLoops')
                obj.TrimPointStruct=setfield(obj.TrimPointStruct,...
                                                    'SLTunerLoops',struct);
            end
            if isfield(obj.TrimPointStruct.SLTunerLoops,...
                                               SLTunerLoop.LoopStruct.Name)
                [Char_array,new_title_row]=obj.getLoopChars(SLTunerLoop);
                SingleLoopTable=cell2table(Char_array,'VariableNames',...
                                           new_title_row);
            else
                obj=obj.AddSLTunerLoops(SLTunerLoop);
                [Char_array,new_title_row]=obj.getLoopChars(SLTunerLoop);
                SingleLoopTable=cell2table(Char_array,'VariableNames',...
                                           new_title_row);
            end
        end
        
        function [CombinedLoopTable]=getCombinedLoopTable(obj,SLTunerLoops)
            CombinedLoopTable=[];
            for i_Loop=1:length(SLTunerLoops)
                SingleLoopTable_i=obj.getLoopTable(SLTunerLoops(i_Loop));
                CombinedLoopTable=[CombinedLoopTable SingleLoopTable_i];
            end
        end
        
        function obj=AddCompleteTable(obj,SLTunerLoops,sltuner_interface)
            FlightConditionTable=obj.getFlightConditionTable;
            if nargin<3
                CoeffsTable=obj.getCoeffsTable;
            else
                CoeffsTable=obj.getCoeffsTable(sltuner_interface);
            end
            CombinedLoopTable=obj.getCombinedLoopTable(SLTunerLoops);
            CompleteTable=[FlightConditionTable CoeffsTable...
                           CombinedLoopTable];
            obj.TrimPointStruct=setfield(obj.TrimPointStruct,...
                                         'CompleteTable',CompleteTable);
        end
            
    end
end
            
            
        
        
    