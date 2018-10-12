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
classdef TrimPointGenerator<handle
    properties
        sltuner_interface
        FlightConditions
        LoopStruct_list
        Options
        TrimPoints
        TrimPoints_Full
    end
    
    methods
        % Constructor
        function obj=TrimPointGenerator(sltuner_interface,...
                                        FlightConditions,LoopStruct_list,...
                                        Options)
            obj.sltuner_interface=sltuner_interface;
            obj.FlightConditions=FlightConditions;
            % Below we call a file that has a number of LoopStructs
            % specific to this task stored. If this becomes an open-source
            % tool we might want to make this more configurable and define
            % the associated LoopStructs somewhere else
            %LoopStructs;
            obj.LoopStruct_list=LoopStruct_list;
            obj.Options=Options;
            obj.TrimPoints=[];
            obj.TrimPoints_Full=0;
        end
        
        function obj=AddTrimPoint(obj,TrimPoint)
            obj.TrimPoints=[obj.TrimPoints TrimPoint];
        end
        
               
                        
        
        % Function that gets every combination of inputs and makes them a
        % row in Input_Combination_Array
        function [TrimPointObj_array]=getTrimPtObjArray(obj)
            
%             alts=sort(obj.FlightConditions.alts);
%             eass=sort(obj.FlightConditions.eass);
%             gammas=sort(obj.FlightConditions.gammas);
%             phis=sort(obj.FlightConditions.phis);

            alts=obj.FlightConditions.alts;
            eass=obj.FlightConditions.eass;
            gammas=obj.FlightConditions.gammas;
            phis=obj.FlightConditions.phis;

            la=length(alts);
            le=length(eass);
            lg=length(gammas);
            lp=length(phis);

            % title_row=cellstr([string('alts') string('eass') 
            % string('gammas') string('phis')]); 
            % Input_Combination_Array=zeros(obj.n_combs,4);
            TrimPointObj_array=[];

            % Now we reorganize the flight conditions into a row for each
            % combination of conditions ordered first based on altitude 
            % followed by eass, gammas, and phis, and place each 
            % combination into a row in the Input_array
            for ia=1:la
                for ie=1:le
                    for ig=1:lg
                        for ip=1:lp
                            index=struct;
                            index.alt_index=ia;
                            index.eas_index=ie;
                            index.gamma_index=ig;
                            index.phi_index=ip;
                            flt=struct;
                            flt.alts=alts(ia);
                            flt.eass=eass(ie);
                            flt.gammas=gammas(ig);
                            flt.phis=phis(ip);
                            TrimPointStruct=struct;
                            TrimPointStruct.indices=index;
                            TrimPointStruct.FlightCondtion=flt;
                            %TrimPointStruct.Loops=struct;
                            TrimPtObj=...
                                    TrimPoint(TrimPointStruct,obj.Options);
                            TrimPointObj_array=[TrimPointObj_array...
                                                TrimPtObj];
                        end
                    end
                end
            end
            
        end
        
        % The following function goes through obj.LoopStruct_list and check
        % if the LoopStructs in the list should be included given the loops
        % in obj.Options.OptConfig.
        % This function is not necessary if we are committed to
        % pre-defining LoopStruct_list as only the necessary loops but it
        % will necessitate downstream changes because this function may be
        % called in other functions. 
        function [SLTunerLoops]=getDesiredLoops(obj)
            SLTunerLoops=[];
            for i=1:length(obj.LoopStruct_list)
                if contains(obj.Options.OptConfig,...
                            obj.LoopStruct_list(i).Name)
                    SLTunerLoops=[SLTunerLoops...
                           SLTunerLoop(obj.LoopStruct_list(i),...
                                obj.sltuner_interface,obj.Options)];
                end
            end
        end
        
        function obj=PopulateTrimPoints(obj)
            % We call the script loops internally here. May want to make it
            % an input to this method i.e.
            % obj=PopulateTrimPoints(obj,file_name), 
            % eval(file_name)
            % file_name would be LoopStructs in this case. There might be 
            % other more configurable ways of accomplishing this
            SLTunerLoops=obj.getDesiredLoops;
            TrimPointObj_array=obj.getTrimPtObjArray;
            for i=1:length(TrimPointObj_array)
                TrimPointObj_array(i).AddCompleteTable(SLTunerLoops,...
                                                    obj.sltuner_interface);
                %does this change the objects in TrimPointObj_array?
                obj=...
                obj.AddTrimPoint(TrimPointObj_array(i).TrimPointStruct);
            end
            obj.TrimPoints_Full=1;
        end
        
        function [CombinedFlightTable]=getCombinedFlightTable(obj)
            CombinedFlightTable=[];
            if obj.TrimPoints_Full
                for i=1:length(obj.TrimPoints)
                    Table_i=obj.TrimPoints(i).CompleteTable;
                    CombinedFlightTable=[CombinedFlightTable;Table_i];
                end
            else
                obj=obj.PopulateTrimPoints;
                for i=1:length(obj.TrimPoints)
                    Table_i=obj.TrimPoints(i).CompleteTable;
                    CombinedFlightTable=[CombinedFlightTable;Table_i];
                end
            end
        end
        
        % The following method is part of the verification test to
        % compare the Crossover Values that the script calculates and the
        % ones that show up in the ViewSpec Plots. This method goes through
        % all of the TrimPoints and for each one goes through all of their
        % subloops and uses SubLoop class methods to determine that the
        % scripts Crossovers fall between the bounding points on the
        % ViewSpec plot. It creates a CrossoverConfirmationStruct, with the
        % property 'Fidelity_indicator' which is 1 if all of the bounding
        % points properly bound a Crossover and 0 if even a single one of
        % them does not.
        function [CrossoverConfirmationStruct]=ConfirmCrossovers(obj)
            CrossoverConfirmationStruct=struct;
            Fidelity_indicator=1;
            if ~obj.TrimPoints_Full
                obj=obj.PopulateTrimPoints;
            end
            
            Failed_TrimPoint_array={};
            names=fieldnames(obj.TrimPoints(1).SLTunerLoops);
            for i_TP=1:length(obj.TrimPoints)
                for j_names=1:length(names)
                    subLoop=getfield(obj.TrimPoints(i_TP).SLTunerLoops,...
                                                     char(names(j_names)));
                    Correct_crossover_booleans=subLoop.isCorrectCrossover;
                    if ismember(0,Correct_crossover_booleans)
                        Fidelity_indicator=0;
                        Failed_TrimPoint_array{1,end+1}=i_TP;
                        Failed_TrimPoint_array{2,end}=names(j_names);
                    end
                end
            end
            CrossoverConfirmationStruct.Fidelity_indicator=...
                                                        Fidelity_indicator;
            CrossoverConfirmationStruct.Failed_TrimPoint_array=...
                                                    Failed_TrimPoint_array;
        end
        
        function []=WriteCharCSV(obj,file_name)
            CombinedFlightTable=obj.getCombinedFlightTable;
            writetable(CombinedFlightTable,file_name);
        end
        
    end
end
                
                
        
        
              