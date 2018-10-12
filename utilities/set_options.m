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
%   PURPOSE:
%   set the options in the current area
%	based on the structure filednames
%
%	INPUTS:
%		optOptions:		Structure containing the function's options must exist
%
%	OUTPUT:
%		variables based on fieldnames of keyowrd structure
%
%--------------------------------------------------------------------------

% Set variables equal to fieldnames in calling function
z_Names = fieldnames(optOptions);
z_Idx=size(z_Names,1);

for i=1:z_Idx
	eval([z_Names{i},' = getfield(optOptions,z_Names{i});'])
end