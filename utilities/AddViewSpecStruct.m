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



% This function's purpose is to add a ViewSpecStruct created by
% GenerateLineStruct to the appropriate element in LoopStruct_list. It will
% be called in the 'OptimizeAquilaClaws_Discrete_GS.m' file after each
% ViewSpec call is made for ST1
function [LoopStruct_list]=...
          AddViewSpecStruct(handle,FltCondition,LoopStruct_list,Loop_name)
    ViewSpecStruct=GenerateLineStructs(handle,FltCondition);
    for i=1:length(LoopStruct_list)
        if contains(LoopStruct_list{i}.Name,Loop_name)
            LoopStruct_list{i}.ViewSpecStruct=ViewSpecStruct;
        end
    end
end
