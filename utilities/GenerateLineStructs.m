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


% This function was written to take in a handle represent the current
% ViewSpec figure and create a struct that can map Data to its respective
% tuning goal and flight condition. 

% Inputs: handle of a ViewSpec figure, Flight conditions

% Extracts all the curves on a viewspec plot associated with a flight
% condition, represents each curve as a pair of data vectors, and places
% each curve in an array object where each element is associated with a
% flight condition. 

% Example: If we are looking at the PitchLoop ViewSpec Plot, if there are 4
% flight conditions then there may be 4 resulting step response curves,
% 4 gain stability margins curves, 4 phase stability margins curves, 
% 4 actual Loop gain curves, 4 sensitivity curve, and 4 complementary 
% sensitivity curves, one curve in each category per flight condition.
% There are also curves representing the target curves in this plot. This
% function creates a ViewSpecStruct for the PitchLoop, which has among
% other fields a field called StepResponse_array. Let's say the flight
% conditions contain 2 alts and 2 eass. Then this array will be a 2X2 array
% where each array element is a struct containing Xdata and Ydata vectors
% for a step response. Element (i,j) would correspond to the ith alt and
% jth eass in the flight conditions and the struct would contain associated
% Info and data for that flight condition's associated step response.

% Outputs: ViewSpecStruct with fields that contain arrays of data vectors
% extracted from ViewSpec plot

% See "Extracting Data From ViewSpec Plots" section of User Manual for more
% instruction

% Developer: David Flamholz

function [ViewSpecStruct]=GenerateLineStructs(handle,FltConditions)
    ViewSpecStruct=struct;
    % The following length help us shape the matrices at the end so that each
    % data structure matches an appropriate flight condition. 
    la=length(FltConditions.alts);
    le=length(FltConditions.eass);
    lg=length(FltConditions.gammas);
    lp=length(FltConditions.phis);

    axesObjs = get(handle, 'Children');   %Get the children of the main handle 
    dataObjs = get(axesObjs, 'Children');
    LineStructs=[];
    StabilityMargins=[];
    %Desired=[];
    Actuals=[];
    %TargetLoopShape=[];
    LoopGains=[];
    Ts=[];
    Ss=[];
    % Note other empty lists can be initiated if we add other tuning goals
    for i_dataObjs=1:length(dataObjs)
        gObject=dataObjs{i_dataObjs};
        for i_gProperty=1:length(gObject)
            if ~isempty(strmatch(gObject(i_gProperty).Type,'hggroup'))
                if isprop(gObject(i_gProperty).Children,'XData')

                    for i_Children=1:length(gObject(i_gProperty).Children)
                        % gObject(i_gProperty).Children sometimes has a length
                        % greater than one, often corresponding to multiple
                        % Line objects. Some of these objects will be 'Curves'
                        % and we will want their data while others whil be
                        % 'StemLines' and we will ignore them
                        LineStruct=struct;
                        LineStruct.XData=...
                           gObject(i_gProperty).Children(i_Children).XData;
                        LineStruct.YData=...
                           gObject(i_gProperty).Children(i_Children).YData;
                        LineStruct.Name=gObject(i_gProperty).DisplayName;
                        LineStruct.XLim=gObject(i_gProperty).Parent.XLim;
                        LineStruct.YLim=gObject(i_gProperty).Parent.YLim;
                        % The following if statement is meant to filter out
                        % empty data structures
                        if ~isnan(LineStruct.XData)
                            if contains(LineStruct.Name,'Stability margins')
                                LineStruct.YLabel=...
                                     gObject(i_gProperty).Parent.YLabel.String;
                                 % This is necessary because there are two
                                 % subplots with different YLabels
                                 % corresponding to the overall Stability
                                 % Margins subplot so we add the property
                                 % YLabel so as to differentiat between the two
                                 % LineStructs
                                StabilityMargins=[LineStruct StabilityMargins];
                            % A number of conditional statements follow. Some 
                            % may need to be added if we add tuning goals. Note
                            % that each LineStruct is put at the front of the
                            % list rather than the back. This is because this
                            % ViewSpec plot orders each set of curves in an
                            % order that is in some sense the reverse of how we
                            % would view them in a single dimension compared to
                            % their indices. For example if we view a set of
                            % flight conditions containing 2 alts and 2
                            % airspeeds as corresponding to a set of indices is
                            % follows: 
                            % [(:,:,1,1) (:,:,1,2) ; (:,:,2,1) (:,:,2,2)]
                            % however corresponding data structs are output in
                            % the following order: 
                            % (:,:,2,2), (:,:,1,2), (:,:,2,1), (:,:,1,1)
                            % Therefore they are reordered and then
                            % appropriately reshaped
                            elseif contains(LineStruct.Name,'Actual')
                                Actuals=[LineStruct Actuals];
                            elseif contains(LineStruct.Name,'Loop gain(s)')
                                LoopGains=[LineStruct LoopGains];
                            elseif ~isempty(strmatch(LineStruct.Name,'T'))
                                Ts=[LineStruct Ts];
                            elseif ~isempty(strmatch(LineStruct.Name,'S'))
                                Ss=[LineStruct Ss];
                            else
                                LineStructs=[LineStruct LineStructs];
                            end
                        end
                    end
                end


            end
        end
    end

    % Now we reshape the lists of structs into matrices of structs where
    % necessary, in which each element corresponds to its appropriate index and
    % therefore flight condition. The Matrices and other structures are placed
    % into the output struct.
    if ~isempty(StabilityMargins)
        ViewSpecStruct.StabilityMargins=StabilityMargins;
    end

    if ~isempty(Actuals)
        ViewSpecStruct.StepResponse_array=reshape(Actuals,la,le,lg,lp);
    end

    if ~isempty(LoopGains)
        ViewSpecStruct.LoopGains_array=reshape(LoopGains,la,le,lg,lp);
    end

    if ~isempty(Ts)
        ViewSpecStruct.T_array=reshape(Ts,la,le,lg,lp);
    end

    if ~isempty(Ss)
        ViewSpecStruct.S_array=reshape(Ss,la,le,lg,lp);
    end

    if ~isempty(LineStructs)
        ViewSpecStruct.LineStructs=LineStructs;
    end


end
