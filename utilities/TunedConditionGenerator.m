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
%   This class creates a Tuned Condition Object for every flight condition
%   and calls downstream class methods to populate that object with relevant
%   info and metrics. The class property, TunedConditions is initialized as
%   an empty list that can lated be populated with TunedConditionStructs
%   (property of TunedCondtion object) enabling us to create a full output
%   table or go a level deeper and analyze each individual tuned condtion
classdef TunedConditionGenerator<handle
    properties
        sltuner_interface
        FlightConditions
        LoopStruct_list
        Options
        TunedConditions
        TunedConditions_Full
    end
    
    methods
        
        function obj=TunedConditionGenerator(sltuner_interface,...
                                         FlightConditions,...
                                         LoopStruct_list, Options)
            obj.sltuner_interface=sltuner_interface;
            obj.FlightConditions=FlightConditions;
            obj.LoopStruct_list=LoopStruct_list;
            obj.Options=Options;
            obj.TunedConditions=[];
            obj.TunedConditions_Full=0;
        end
        
        
        
        
        
        
        % The purpose of this function is to recreate a set of LoopStructs that
        % organize the Tuning goal characteristics into associated structs that can
        % feed the necessary nodes, names, openings, goals and objectives into the
        % downstream classes to create appropriate transfer functions corresponding
        % to each tuned condition.
        function [LoopStruct]=...
               ReMakeLoopStruct(obj, Gain_sign, index)
            LoopStruct=obj.LoopStruct_list{index};
            TuningGoals_list=LoopStruct.TuningGoals_list;
            LoopStruct.Loop_Gain_sign=Gain_sign;
            Margins_counter=1;


            for i=1:length(TuningGoals_list)
                if isa(TuningGoals_list(i),'TuningGoal.LoopShape')
                    LoopStruct.Loop_Gain_openings=TuningGoals_list(i).Openings';
                    LoopStruct.Loop_Gain_node=TuningGoals_list(i).Location{1};
                    
                    LoopStruct.Lgroup=...
                        obj.sltuner_interface.getLoopTransfer(LoopStruct.Loop_Gain_node,...
                                           LoopStruct.Loop_Gain_sign,...
                                           LoopStruct.Loop_Gain_openings);
                    
                    
                elseif isa(TuningGoals_list(i),'TuningGoal.StepTracking')
                    LoopStruct.CL_input_node=TuningGoals_list(i).Input;
                    LoopStruct.CL_output_node=TuningGoals_list(i).Output;
                    LoopStruct.CL_openings=TuningGoals_list(i).Openings';
                    
                    LoopStruct.CLgroup=...
                         obj.sltuner_interface.getIOTransfer(LoopStruct.CL_input_node,...
                                          LoopStruct.CL_output_node,...
                                          LoopStruct.CL_openings);
                                        
                    
                elseif isa(TuningGoals_list(i),'TuningGoal.Margins')
                    if Margins_counter==1
                        LoopStruct.StabilityMargins=struct;
                        LoopStruct.StabilityMargins.Input=struct;
                        LoopStruct.StabilityMargins.Input.Node=...
                                                       TuningGoals_list(i).Location{1};
                        LoopStruct.StabilityMargins.Input.Openings=...
                                                         TuningGoals_list(i).Openings';                        
                        
                        Input=LoopStruct.StabilityMargins.Input;
                        LoopStruct.MarginsInputTFgroup=...
                               obj.sltuner_interface.getLoopTransfer(Input.Node,...
                                                  LoopStruct.Loop_Gain_sign,...
                                                  Input.Openings);
                        Margins_counter=Margins_counter+1;
                        
                        
                    elseif Margins_counter==2
                        LoopStruct.StabilityMargins.Output=struct;
                        LoopStruct.StabilityMargins.Output.Node=...
                                                       TuningGoals_list(i).Location{1};
                        LoopStruct.StabilityMargins.Output.Openings=...
                                                         TuningGoals_list(i).Openings';                       
                        
                        Output=LoopStruct.StabilityMargins.Output;
                        LoopStruct.MarginsOutputTFgroup=...
                               obj.sltuner_interface.getLoopTransfer(Output.Node,...
                                                  LoopStruct.Loop_Gain_sign,...
                                                  Output.Openings);                                         
                    end
                    
                    %% Add more info here
                elseif isa(TuningGoals_list(i),'TuningGoal.Sensitivity')
                    %LoopStruct.Sensitivity_openings=TuningGoals_list(i).Openings';
                    % to calculate the sensitvity function broken at the 
                    % measure output signal, leave all the loops closed
                    LoopStruct.Sensitivity_openings={}; 
                    LoopStruct.Sensitivity_node=TuningGoals_list(i).Location{1};
                    
                    LoopStruct.SensitivityTFgroup=...
                        obj.sltuner_interface.getSensitivity(LoopStruct.Sensitivity_node,...
                                                          LoopStruct.Sensitivity_openings);
                
                end
            end
        end
        
        
        
        
        function obj=RebuildLoopStruct_list(obj)
            for index=1:length(obj.LoopStruct_list)
                obj.LoopStruct_list{index}=obj.ReMakeLoopStruct(-1, index);
            end
        end
        
        
        function obj=AddTunedCondition(obj,TunedCondition)
            obj.TunedConditions=[obj.TunedConditions TunedCondition];
        end
        
               
                        
        
        % Function that gets every combination of inputs and makes them a
        % row in Input_Combination_Array
        function [TunedConditionObj_array]=getTunedCondObjArray(obj)
            
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

            TunedConditionObj_array=[];

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
                            TunedConditionStruct=struct;
                            TunedConditionStruct.indices=index;
                            TunedConditionStruct.FlightCondtion=flt;
                            TunedConditionObj=...
                          TunedCondition(TunedConditionStruct,obj.Options);
                            TunedConditionObj_array=...
                                                [TunedConditionObj_array...
                                                 TunedConditionObj];
                        end
                    end
                end
            end
            
        end
        
        % The following function creates a list of SLTunerLoops based on
        % the LoopStructs in LoopStruct_list
        function [SLTunerLoops]=getDesiredLoops(obj)
            SLTunerLoops=[];
            for i=1:length(obj.LoopStruct_list)
                SLTunerLoops=[SLTunerLoops...
                              SLTunerLoop(obj.LoopStruct_list{i},...
                                       obj.sltuner_interface,obj.Options)];
            end
        end
        
        function obj=PopulateTunedConditions(obj)
            obj=obj.RebuildLoopStruct_list;
            SLTunerLoops=obj.getDesiredLoops;
            TunedConditionObj_array=obj.getTunedCondObjArray;
            for i=1:length(TunedConditionObj_array)
               TunedConditionObj_array(i).AddCompleteTable(SLTunerLoops,...
                                                    obj.sltuner_interface);
                
                obj=...
                obj.AddTunedCondition(TunedConditionObj_array(i).TunedConditionStruct);
            end
            obj.TunedConditions_Full=1;
        end
        
        function [CombinedFlightTable]=getCombinedFlightTable(obj)
            CombinedFlightTable=[];
            if ~obj.TunedConditions_Full
                obj=obj.PopulateTunedConditions;
            end
            
            for i=1:length(obj.TunedConditions)
                Table_i=obj.TunedConditions(i).CompleteTable;
                CombinedFlightTable=[CombinedFlightTable;Table_i];
            end
        end
        
        % The following method is part of the verification test to
        % compare the Crossover Values that the script calculates and the
        % ones that show up in the ViewSpec Plots. This method goes through
        % all of the TunedConditions and for each one goes through all of 
        % their subloops and uses SubLoop class methods to determine that 
        % the scripts Crossovers fall between the bounding points on the
        % ViewSpec plot. It creates a CrossoverConfirmationStruct, with the
        % property 'Fidelity_indicator' which is 1 if all of the bounding
        % points properly bound a Crossover and 0 if even a single one of
        % them does not.
        function [CrossoverConfirmationStruct]=ConfirmCrossovers(obj)
            CrossoverConfirmationStruct=struct;
            Fidelity_indicator=1;
            if ~obj.TunedConditions_Full
                obj=obj.PopulateTunedConditions;
            end
            
            Failed_TunedCondition_array={};
            names=fieldnames(obj.TunedConditions(1).SLTunerLoops);
            for i_TC=1:length(obj.TunedConditions)
                for j_names=1:length(names)
                    subLoop=getfield(obj.TunedConditions(i_TC).SLTunerLoops,...
                                                     char(names(j_names)));
                    Correct_crossover_booleans=subLoop.isCorrectCrossover;
                    if ismember(0,Correct_crossover_booleans)
                        Fidelity_indicator=0;
                        Failed_TunedCondition_array{1,end+1}=i_TC;
                        Failed_TunedCondition_array{2,end}=names(j_names);
                    end
                end
            end
            CrossoverConfirmationStruct.Fidelity_indicator=...
                                                        Fidelity_indicator;
            CrossoverConfirmationStruct.Failed_TunedCondition_array=...
                                               Failed_TunedCondition_array;
        end
        
        function [StepInfoConfirmationStruct]=ConfirmStepInfo(obj)
            StepInfoConfirmationStruct=struct;
            Fidelity_indicator=1;
            if ~obj.TunedConditions_Full
                obj=obj.PopulateTunedConditions;
            end
            Failed_TunedCondition_array={};
            names=fieldnames(obj.TunedConditions(1).SLTunerLoops);
            for i_TC=1:length(obj.TunedConditions)
                for j_names=1:length(names)
                    subLoop=getfield(obj.TunedConditions(i_TC).SLTunerLoops,...
                                                     char(names(j_names)));
                    if ~subLoop.isCorrectStepInfo
                        Fidelity_indicator=0;
                        Failed_TunedCondition_array{1,end+1}=i_TC;
                        Failed_TunedCondition_array{2,end}=names(j_names);
                    end
                end
            end
            StepInfoConfirmationStruct.Fidelity_indicator=...
                                                        Fidelity_indicator;
            StepInfoConfirmationStruct.Failed_TunedCondition_array=...
                                               Failed_TunedCondition_array;
        end
        
        % The purpose of this method is to Check each one of the
        % parameters/coeffs that were extracted from the gain surface plots
        % against the corresponding parameter in the associated
        % TunedConditionStruct.Coeffs. Note, we can only check
        % GainSurface info if we are Optimizing for multiple flight
        % conditions
        function [ParamConfirmationStruct]=ConfirmParameters(obj)
            if obj.Options.Optimize4MultipleFlts
                ParamConfirmationStruct=struct;
                Fidelity_indicator=1;
                if ~obj.TunedConditions_Full
                    obj=obj.PopulateTunedConditions;
                end
                Failed_Parameter_array={};

                for i_LS=1:length(obj.LoopStruct_list)
                    LoopStruct=obj.LoopStruct_list{i_LS};
                    for j_GS=1:length(LoopStruct.GainSurfaceStructs)
                        GainStruct=LoopStruct.GainSurfaceStructs(j_GS);
                        m=size(GainStruct.Coeffs,1);
                        n=size(GainStruct.Coeffs,2);
                        for i=1:m
                            for j=1:n
                                Surf_coeff=GainStruct.Coeffs(i,j);
                                % below we get the index in obj.TunedConditions
                                % associated with the (i,j) element of the
                                % Coeffs array
                                i_TC=n*(i-1)+j;
                                TC=obj.TunedConditions(i_TC);
                                TC_coeff=getfield(TC.Coeffs,GainStruct.Name);
                                if ~CloseEnough(Surf_coeff,TC_coeff)
                                    Fidelity_indicator=0;
                                    Failed_Parameter_array{1,end+1}=i_TC;
                                    Failed_Parameter_array{2,end}=...
                                                               GainStruct.Name;
                                end
                            end
                        end

                    end
                end
                ParamConfirmationStruct.Fidelity_indicator=Fidelity_indicator;
                ParamConfirmationStruct.Failed_Parameter_array=...
                                                        Failed_Parameter_array;
            else
                 ParamConfirmationStruct=struct;
                 ParamConfirmationStruct.Fidelity_indicator=1;
                 ParamConfirmationStruct.Failed_Parameter_array={};
            end
        end
        
        % The purpose of this function is to confirm that there is
        % agreement between each TunedConditionStructs associated parameters,
        % and performance characteristics and those of the surface and
        % viewspec plots
        function [CompleteConfirmationStruct]=ConfirmAllParameters(obj)
            CompleteConfirmationStruct=struct;
            CrossoverConfirmationStruct=obj.ConfirmCrossovers;
            StepInfoConfirmationStruct=obj.ConfirmStepInfo;
            ParamConfirmationStruct=obj.ConfirmParameters;
            Fidelity_indicator=0;
            if (CrossoverConfirmationStruct.Fidelity_indicator...
                &&...
                ParamConfirmationStruct.Fidelity_indicator...
                &&...
                StepInfoConfirmationStruct.Fidelity_indicator)
                Fidelity_indicator=1;
            end
            CompleteConfirmationStruct.Fidelity_indicator=...
                                                        Fidelity_indicator;
            CompleteConfirmationStruct.CrossoverConfirmationStruct=...
                                               CrossoverConfirmationStruct;
            CompleteConfirmationStruct.StepInfoConfirmationStruct=...
                                                StepInfoConfirmationStruct;
            CompleteConfirmationStruct.ParamConfirmationStruct=...
                                                   ParamConfirmationStruct;
        end
        
        % include .csv or .xls in file_name
        function []=WriteCharCSV(obj,file_name)
            CombinedFlightTable=obj.getCombinedFlightTable;
            writetable(CombinedFlightTable,file_name);
        end
        
        % The purpose of this function is to create a struct that carries
        % all of the parameters associated with a tuned condition
        % that show up in the output csv. This will be used in ConfirmCSV
        function [TC_Param]=getTunedConditionParam(obj,TunedConditionStruct)
            TC_Param=struct;
            
            Flt_names=fieldnames(TunedConditionStruct.FlightCondtion);
            
            for i_flt=1:length(Flt_names)
                flt_cond=getfield(TunedConditionStruct.FlightCondtion,char(Flt_names(i_flt)));
                TC_Param=setfield(TC_Param,char(Flt_names(i_flt)),flt_cond);
            end
                
            Coeff_names=fieldnames(TunedConditionStruct.Coeffs);
            
            for i_Coeff=1:length(Coeff_names)
                Coeff=getfield(TunedConditionStruct.Coeffs,char(Coeff_names(i_Coeff)));
                TC_Param=setfield(TC_Param,char(Coeff_names(i_Coeff)),Coeff);
            end
            
            STLoops=TunedConditionStruct.SLTunerLoops;
            Loop_names=fieldnames(STLoops);
            
            for i_loop=1:length(Loop_names)
                subLoop=getfield(STLoops,char(Loop_names(i_loop)));
                StepStruct=subLoop.getStepStruct(Loop_names(i_loop));
                step_names=fieldnames(StepStruct);
                
                for i_step=1:length(step_names)
                    step_val=getfield(StepStruct,char(step_names(i_step)));
                    TC_Param=setfield(TC_Param,char(step_names(i_step)),step_val);
                end
                
                crossover_name=strcat(Loop_names(i_loop),'_Crossover');
                if obj.Options.getAllCrossovers
                    crossover=subLoop.Crossover;
                    if length(crossover)==1
                        TC_Param=setfield(TC_Param,char(crossover_name),crossover);
                    else
                        for i_c=1:length(crossover)
                            new_crossover_name=strcat(crossover_name,'_',num2str(i_c));
                            TC_Param=setfield(TC_Param,char(new_crossover_name),crossover(i_c));
                        end
                    end
                else
                    crossover=subLoop.Last_crossover;
                    TC_Param=setfield(TC_Param,char(crossover_name),crossover);
                end
            end
        end
        
        
        % This method uses the previous method to get a list of TC_Param
        % structs. Each TC_Param struct in the list corresponds to one at
        % the same location in obj.TunedConditions
        function [TC_Param_list]=getTunedConditionParamList(obj)
            if ~obj.TunedConditions_Full
                obj=obj.PopulateTunedConditions;
            end
            TC_Param_list={};
            for i_TC=1:length(obj.TunedConditions)
                TC_Param=obj.getTunedConditionParam(obj.TunedConditions(i_TC));
                TC_Param_list{end+1}=TC_Param;
            end
        end
        
        % include .csv or .xls in file_name
        function [CsvConfirmationStruct]=ConfirmCSV(obj,file_name)
            if ~obj.TunedConditions_Full
                obj=obj.PopulateTunedConditions;
            end
            Fidelity_indicator=1;
            Failed_Parameter_array={};
            CsvConfirmationStruct=struct;
            CompleteConfirmationStruct=obj.ConfirmAllParameters;
            if ~CompleteConfirmationStruct.Fidelity_indicator
                CsvConfirmationStruct.Fidelity_indicator=0;
            else
                tab=readtable(file_name);
                [m,n]=size(tab);
                TC_Param_list=obj.getTunedConditionParamList;
                for i=1:m
                    TC_Param=TC_Param_list{i};
                    %i_crossover=0;
                    for j=1:n
                        element=tab(i,j);
                        name=fieldnames(element);
                        name=name(1);
                        CSV_val=getfield(element,char(name));
              
                        if contains(name,'Crossover_1')
                            new_name=reverse(char(name));
                            new_name=new_name(find(new_name=='_')+1:end);
                            new_name=reverse(new_name);

                            TC_val=[];
                            if ismember(1,contains(fieldnames(TC_Param),name))
                                TC_name=name;
                                TC_val=getfield(TC_Param,char(TC_name));
                            elseif ismember(1,contains(fieldnames(TC_Param),new_name))
                                TC_name=new_name;
                                TC_val=getfield(TC_Param,char(TC_name));
                            end
                            
                            if ~isempty(TC_val)
                                if ~CloseEnough(CSV_val,TC_val)
                                    Fidelity_indicator=0;
                                    Failed_Parameter_array{1,end+1}=i;
                                    Failed_Parameter_array{2,end}=TC_name;
                                end
                            end
                        else
                            if ismember(1,contains(fieldnames(TC_Param),name))
                                TC_val=getfield(TC_Param,char(name));
                                if ~CloseEnough(CSV_val,TC_val)
                                    Fidelity_indicator=0;
                                    Failed_Parameter_array{1,end+1}=i;
                                    Failed_Parameter_array{2,end}=name;
                                end
                            end
                        end
                        
                    end
                end
            end
            CsvConfirmationStruct.Fidelity_indicator=Fidelity_indicator;
            CsvConfirmationStruct.CompleteConfirmationStruct=...
                                                CompleteConfirmationStruct;
            CsvConfirmationStruct.Failed_Parameter_array=Failed_Parameter_array;
        end
            
        
    end
end
                       