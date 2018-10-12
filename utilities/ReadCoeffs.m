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
%   Developer: David Flamholz
%   The purpose of this function is to extract the different gains and 
%   lead/lag coefficients off of a current figure and create a data structure
%   with this info. The Structure will contain a property 'Coeffs' which is
%   an array of the parameters with each elemental index corresponding to an
%   associated flight condition
function [GainSurfaceStruct]=ReadCoeffs(handle)
axesObjs = get(handle, 'Children');
dataObjs = get(axesObjs, 'Children');
GainSurfaceStruct=struct;
GainSurfaceStruct.Coeffs=dataObjs.ZData;
GainSurfaceStruct.Name=axesObjs.Title.String;
end
