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
%   Each Tuned Condition is an object that contains all the information in
%   one row of the output table as well as other metrics and methods, the
%   user may want to further analyze and process the data
classdef TunedCondition<handle
    properties
        % TunedConditionStruct intially contains a single flight condtion 
        % and an associated index. We will add fields to it using class
        % methods and place each populated TunedCondition in a list
        % belonging to the upstream class TunedCondition Generator
        TunedConditionStruct
        Options
        
    end
    
    
    methods
        % Constructor
        function obj=TunedCondition(TunedConditionStruct,Options)
            obj.TunedConditionStruct=TunedConditionStruct;
            obj.Options=Options;
        end
        
        function [FlightConditionTable]=getFlightConditionTable(obj)
            alts=obj.TunedConditionStruct.FlightCondtion.alts;
            eass=obj.TunedConditionStruct.FlightCondtion.eass;
            gammas=obj.TunedConditionStruct.FlightCondtion.gammas;
            phis=obj.TunedConditionStruct.FlightCondtion.gammas;
            FlightConditionTable=table(alts,eass,gammas,phis);
        end
        
        
        function obj=AddSLTunerLoops(obj,SLTunerLoop)
            if ~isfield(obj.TunedConditionStruct,'SLTunerLoops')
                obj.TunedConditionStruct=...
                                      setfield(obj.TunedConditionStruct,...
                                               'SLTunerLoops',struct);
            end
            Loop_name=SLTunerLoop.LoopStruct.Name;

            SubLoopStruct=SLTunerLoop.getSubLoopStruct(obj.TunedConditionStruct.indices);
            
            subLoop=SubLoop(SubLoopStruct);
            subLoop=subLoop.PopulateTable;
            
            obj.TunedConditionStruct.SLTunerLoops=...
                         setfield(obj.TunedConditionStruct.SLTunerLoops,...
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
                        obj.TunedConditionStruct.indices.gamma_index,...
                        obj.TunedConditionStruct.indices.phi_index,...
                        obj.TunedConditionStruct.indices.alt_index,...
                        obj.TunedConditionStruct.indices.eas_index);
                    Coeffs=setfield(Coeffs,ppty_name,val);
                end
 
            else
                for i_n=1:n_fields
                    final_field_i=...
                               getfield(BlockParameters,char(fields(i_n)));
                    ppty_name=final_field_i.Name;
                    val=final_field_i.Value;
                    Coeffs=setfield(Coeffs,ppty_name,val);
                end
                
            end
            obj.TunedConditionStruct=...
                                  setfield(obj.TunedConditionStruct,...
                                           'Coeffs',Coeffs);
            obj.TunedConditionStruct.Coeffs=...
                              orderfields(obj.TunedConditionStruct.Coeffs);
        end
        
        
        function [CoeffsTable]=getCoeffsTable(obj,sltuner_interface)
            if nargin<2
                CoeffsTable=struct2table(obj.TunedConditionStruct.Coeffs);
            else
                if isfield(obj.TunedConditionStruct,'Coeffs')
                    CoeffsTable=...
                             struct2table(obj.TunedConditionStruct.Coeffs);
                else
                    obj=obj.AddCoeffs(sltuner_interface);
                    CoeffsTable=...
                             struct2table(obj.TunedConditionStruct.Coeffs);
                end
            end
        end
                
                
        % Creates a table of characteristics associated with a single loop
        % e.g. obj.TunedConditionStruct.SLTunerLoops.PitchLoop. This table can be
        % combined with the tables associated with other loops as well as a
        % coeffs table to get a combined table associated with each flight
        % condition
        function [SingleLoopTable]=getLoopTable(obj,SLTunerLoop)
            if ~isfield(obj.TunedConditionStruct,'SLTunerLoops')
                obj.TunedConditionStruct=...
                                      setfield(obj.TunedConditionStruct,...
                                               'SLTunerLoops',struct);
            end
            if ~isfield(obj.TunedConditionStruct.SLTunerLoops,...
                                               SLTunerLoop.LoopStruct.Name)
                obj=obj.AddSLTunerLoops(SLTunerLoop);
            end
            Loop_name=SLTunerLoop.LoopStruct.Name;
            subLoop=getfield(obj.TunedConditionStruct.SLTunerLoops,Loop_name);
            SingleLoopTable=subLoop.PerformanceTable;
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
            obj.TunedConditionStruct=setfield(obj.TunedConditionStruct,...
                                         'CompleteTable',CompleteTable);
        end
            
    end
end
            
            
        
        
    