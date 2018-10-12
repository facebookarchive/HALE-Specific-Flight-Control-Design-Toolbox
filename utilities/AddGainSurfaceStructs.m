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


% The following function has the purpose of Adding a property 
% 'SurfaceCoeffsStructs' to a LoopStruct in LoopStruct_List. This Property
% will be a list. The if the property already exists. The function then
% adds the appropriate SurfaceCoeffsStruct, gotten from ReadCoeffs to the
% list. This will be implemented in the MultipleFlightCondtions portion of
% the 'OptimizeAquilaClaws_Discrete_GS.m' script in the Loop where all the
% gain Surfaces are created.
function [LoopStruct_list]=AddGainSurfaceStructs(handle,LoopStruct_list)
GainSurfaceStruct=ReadCoeffs(handle);
for i=1:length(LoopStruct_list)
    if contains(GainSurfaceStruct.Name,LoopStruct_list{i}.Name)
        if ~isfield(LoopStruct_list{i},'GainSurfaceStructs')
            % The following will become a populated list of 
            % SurfaceCoeffsStructs, one for each parameter pertaining to
            % the LoopStruct in question (i.e. PitchLoop or ClimbRateLoop)
            LoopStruct_list{i}.GainSurfaceStructs=[];
        end
        LoopStruct_list{i}.GainSurfaceStructs=...
             [LoopStruct_list{i}.GainSurfaceStructs GainSurfaceStruct];
    end
end
end
