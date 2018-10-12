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
%   Unlike the general SLTunerLoop Class this class describes the Loop 
%   structures that would be associated with a single flight condition  
%   and therefore Tuned Condition. It is initiated with a SubLoopStruct that
%   contains CL, L, MarginTFs, a LoopStruct, and Options. Other things can be
%   added to the SubLoopStruct in SLTunerLoop if we want to be able to
%   calculate and present more.
%   We can then use SubLoop class methods to operate on
%   these individual transfer functions and get data and other information.
%   These SubLoops are associated with an individual TunedCondition which 
%   will be specified for a single set of flight conditions. We can see the
%   downstream structure as follows where PitchLoop is the example of an
%   instantiation of the SubLoop class:
%   TunedCondition.SLTunerLoops.PitchLoop.CL or
%   TunedCondition.SLTunerLoops.PitchLoop.Crossover
%   Thus, this class enables us to oraganize lower level data associated with
%   each TunedCondition

classdef SubLoop
    properties
        SubLoopStruct
        CL
        L
        MarginsInputTF
        MarginsOutputTF
        ViewSpecSubStruct
        StepInfo
        Crossover
        Last_crossover
        Name
        PerformanceTable
    end
    
    methods
        
        % Constructor
        function obj=SubLoop(SubLoopStruct)
            obj.SubLoopStruct=SubLoopStruct;
            
            if isfield(SubLoopStruct,'CL')
                obj.CL=SubLoopStruct.CL;
            else
                obj.CL=[];
            end
            
            if isfield(SubLoopStruct,'L')
                obj.L=SubLoopStruct.L;
            else
                obj.L=[];
            end
            
            if isfield(SubLoopStruct,'MarginsInputTF')
                obj.MarginsInputTF=SubLoopStruct.MarginsInputTF;
            else
                obj.MarginsInputTF=[];
            end
            
            if isfield(SubLoopStruct,'MarginsOutputTF')
                obj.MarginsOutputTF=SubLoopStruct.MarginsOutputTF;
            else
                obj.MarginsOutputTF=[];
            end
            
            if isfield(SubLoopStruct,'ViewSpecSubStruct')
                obj.ViewSpecSubStruct=SubLoopStruct.ViewSpecSubStruct;
            else
                obj.ViewSpecSubStruct=[];
            end
      
            
            if ~isempty(obj.CL)
                obj.StepInfo=stepinfo(obj.CL);
            else
                obj.StepInfo=[];
            end
            
            if ~isempty(obj.L)
                Crossovers=(getGainCrossover(obj.L,1))';
            else
                Crossovers=[];
            end
            obj.Crossover=Crossovers;
            
            if ~isempty(obj.Crossover)
                obj.Last_crossover=obj.Crossover(end);
            else
                obj.Last_crossover=[];
            end
            
            obj.Name=obj.SubLoopStruct.LoopStruct.Name;
            
            obj.PerformanceTable=[];
            
        end
        
        % Function that enables you to add metrics to output table
        % Inputs: 'metrics' & 'names' should be  equal length 
        % cell arrays with characteristic numbers or other characteristics
        % in 'metrics', and corresponding titles in 'names'
        function obj=AddMetricToTable(obj,metrics,names)
            Intro_name=strcat(obj.Name,'_');
            Intro_name_array=cell(1,length(names));
            for i=1:length(names)
                Intro_name_array{i}=strcat(Intro_name,names{i});
            end
            NewTable=cell2table(metrics,'VariableNames',Intro_name_array);
            obj.PerformanceTable=[obj.PerformanceTable NewTable];
        end
        
        % Here we begin defining methods for Adding specific metrics to the
        % output table. For example, to start, we will have a function that
        % adds Step Response metrics to the table. The result will be that
        % the table will have extra columns with 'Overshoot','RiseTime',
        % and any other specified Step response characteristics in the
        % title row and associated metrics down the column for each Tuned
        % Condition
        function obj=AddStepResponseMetrics(obj)
            % Specify step response characteristics that can be produced
            % from stepinfo command in names below to get the ones you want
            % to show up in table
            names={'Overshoot','RiseTime','SettlingTime','Peak','PeakTime'};
            metrics=cell(1,length(names));
            for i=1:length(names)
                metrics{i}=getfield(obj.StepInfo,names{i});
            end
            obj=obj.AddMetricToTable(metrics,names);
        end
        
        function obj=AddFrequencyResponseMetrics(obj)
            names={'Crossover','Classical_GainMargin_dB',...
                   'Classical_PhaseMargin','Tracking_Limit_Mag_dB',...
                   'Noise_Rejection_Mag_dB'};
            metrics=cell(1,length(names));
            
            if obj.SubLoopStruct.Options.getAllCrossovers
                metrics{1}={obj.Crossover};
            else
                metrics{1}=obj.Last_crossover;
            end
            
            [Gm,Pm,Wgm,Wpm]=margin(obj.L);
            Gm_dB=20*log10(Gm); %Gain Margin in units of dB
            metrics{2}=Gm_dB;
            metrics{3}=Pm;
            
            % Specify w_vec in rad/sec-- This is already done in our case
            w_vec=[0.1*obj.Last_crossover 10*obj.Last_crossover]; 
            [mag,phase,wout] = bode(obj.L,w_vec);
            mag=reshape(mag,length(wout),1);
            mag_dB=mag2db(mag);
            metrics{4}=mag_dB(1);
            metrics{5}=mag_dB(2);
            obj=obj.AddMetricToTable(metrics,names);
        end
        
        %%
        % Add Method here to Add values to obj.PerformanceTable like the 
        % two above. Then call that method in PopulateTable below.
        % This will result in added columns with the desired metrics in 
        % output csv file
        %
        
        function obj=AddSensitivityMetrics(obj)
            if isfield(obj.SubLoopStruct,'SensitivityTF')
                names={'PeakSensitivity_db','PeakSensitivity_Frequency'};
                metrics=cell(1,length(names));
                SenseTF=obj.SubLoopStruct.SensitivityTF;
                [gpeak,fpeak] = getPeakGain(SenseTF);
                gpeak=mag2db(gpeak);
                metrics{1}=gpeak;
                metrics{2}=fpeak;
                obj=obj.AddMetricToTable(metrics,names);
            end
        end
        
        % The following method populates obj.PerformanceTable with
        % performance metrics associated with this subloop which will later
        % be filled into an overall table associated with a TunedCondition.
        % If you want to add something to the table, create a method like
        % the ones above which call obj.AddMetricToTable and then call that
        % method in the function below
        function obj=PopulateTable(obj)
            obj=obj.AddStepResponseMetrics;
            obj=obj.AddFrequencyResponseMetrics;
            %% Call new SubLoop Add_ methods here to add metrics to the output table
            obj=obj.AddSensitivityMetrics;
        end
                 
        
        
        % Method to get Step Response as y vs. t data
        function [y,t]=getStepResponseData(obj,Tfinal)
            if nargin<2
                [y,t]=step(obj.CL);
            else
                [y,t]=step(obj.CL,Tfinal);
            end
        end
        
        % Method to get Frequency Response as Mag vs. w Data and phase vs.
        % w data
        function [mag_dB, phase, wout]=getFrequencyResponseData(obj,w)
            if nargin<2
                [mag,phase,wout] = bode(obj.L);
            else
                [mag,phase,wout] = bode(obj.L,w);
            end
            mag=reshape(mag,length(wout),1);
            mag_dB=mag2db(mag); %bode command gives mag in absolute units
            phase=reshape(phase,length(wout),1); %bode command gives phase in degrees
        end
        
        % Method to get Classical gain and phase margin. This is not
        % necessarily the margin that shows up on the ViewSpec Margins plot
        function [Gm_dB,Pm,Wgm,Wpm]=getClassicalMargins(obj)
            [Gm,Pm,Wgm,Wpm]=margin(obj.L);
            Gm_dB=20*log10(Gm); %Outputs Gain Margin in units of dB
        end
        
        
        
        % This function take as LoopGain struct containing properties XData
        % and YData and goes through that discrete data to find the pairs 
        % of frequency points that bound the crossover frequency from above
        % and below by looking for the corresponding Magnitude points in
        % YData that bound the 0 db line from above and below
        function [Upper_bounds,Lower_bounds]=GetCrossoverBounds(obj)
            if isfield(obj.ViewSpecSubStruct,'LoopGainData')
                w=obj.ViewSpecSubStruct.LoopGainData.XData;
                Mag=obj.ViewSpecSubStruct.LoopGainData.YData;
                XLim=obj.ViewSpecSubStruct.LoopGainData.XLim;
                indices=find(w>=XLim(1)&w<=XLim(2));
                w=w(indices);
                Mag=Mag(indices);
                Lower_bounds=[];
                Upper_bounds=[];
                for i=1:(length(Mag)-1)
                    if Mag(i)>0 && Mag(i+1)<0
                        Upper_bounds=[Upper_bounds w(i)];
                        Lower_bounds=[Lower_bounds w(i+1)];
                    elseif Mag(i)<0 && Mag(i+1)>0
                        Upper_bounds=[Upper_bounds w(i+1)];
                        Lower_bounds=[Lower_bounds w(i)];
                    end
                end
            else
                Upper_bounds=[];
                Lower_bounds=[];
            end
        end
        
        % This function looks at a given SubLoop with the name Loop_name
        % and checks whether the crossover data extracted from the
        % corresponding ViewSpec plot agrees with the crossover we
        % calculated from the L transfer function associated with this
        % obj.TrimPointStruct
        function [Correct_crossover_booleans]=isCorrectCrossover(obj)
            [Upper_bounds,Lower_bounds]=obj.GetCrossoverBounds;
            if isempty(Upper_bounds)
                Correct_crossover_booleans=[];
            
            else
                Correct_crossover_booleans=zeros(1,length(Upper_bounds));
                XLim=obj.ViewSpecSubStruct.LoopGainData.XLim;
                indices=find(obj.Crossover>=XLim(1)&obj.Crossover<=XLim(2));
                Crossovers=obj.Crossover(indices);
                for i=1:length(Upper_bounds)
                    
                    if ((Upper_bounds(i)<=Crossovers(i) && ...
                            Crossovers(i)<=Lower_bounds(i))...
                            ||...
                            (Lower_bounds(i)<=Crossovers(i) &&...
                            Crossovers(i)<=Upper_bounds(i)))
                        
                        Correct_crossover_booleans(i)=1;
                    end
                end
            end
        end
        
        function [Correct_CL_boolean]=isCorrectCL(obj)
            if isfield(obj.ViewSpecSubStruct,'LoopGainData')
                XLim=obj.ViewSpecSubStruct.StepResponseData.XLim;
                %YLim=obj.ViewSpecSubStruct.StepResponseData.YLim;
                Xdata1=obj.ViewSpecSubStruct.StepResponseData.XData;
                Ydata1=obj.ViewSpecSubStruct.StepResponseData.YData;
                indices=find(Xdata1>=XLim(1)&Xdata1<=XLim(2));
                VS_XData=Xdata1(indices);
                VS_YData=Ydata1(indices);
                [VS_XData,ia,ic]=unique(VS_XData);
                VS_YData=VS_YData(ia);
                SubLoop_YData=step(obj.CL,VS_XData);
                Correct_CL_boolean=1;
                for i=1:length(VS_XData)
                    if ~CloseEnough(VS_YData(i),SubLoop_YData(i))
                        Correct_CL_boolean=0;
                        break
                    end
                end
            else
                Correct_CL_boolean=1;
            end

        end
        
        function [Step_info_boolean]=isCorrectStepInfo(obj)
            Step_info_boolean=1;
            if obj.isCorrectCL
                Step_info=stepinfo(obj.CL);
                step_names=fieldnames(Step_info);
                for i=1:length(step_names)
                    Test_char=getfield(Step_info,char(step_names(i)));
                    Subloop_char=getfield(obj.StepInfo,char(step_names(i)));
                    if ~CloseEnough(Test_char,Subloop_char)
                        Step_info_boolean=0;
                        break;
                    end
                end
            else
                Step_info_boolean=0;
            end
        end
        
        function [StepStruct]=getStepStruct(obj,Loop_name)
            StepStruct=struct;
            Step_names=fieldnames(obj.StepInfo);
            for i=1:length(Step_names)
                new_name=strcat(Loop_name,'_',Step_names(i));
                Step_char=getfield(obj.StepInfo,char(Step_names(i)));
                StepStruct=setfield(StepStruct,char(new_name),Step_char);
            end
        end
        
    end
end