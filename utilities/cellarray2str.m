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



% SYNTAX:		[Str] = cellarray2str(CellArray)
% 
% PURPOSE:		Convert cell array to a character array with ',' as delimiter.
% 


function [Str] = cellarray2str(CellArray)

Delimiter	= ',';
NCells		= length(CellArray);
Str			= '';
for iCell = 1:NCells
	Str	= [Str,Delimiter,CellArray{iCell}];
end
Str		= Str(2:end);	% chop first delimiter