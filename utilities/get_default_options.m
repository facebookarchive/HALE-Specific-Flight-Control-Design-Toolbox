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


function [option_out] = get_default_options(option_in,defaults)
%   Purpose:
%   This function processes the options provided to a function
%	against supplied default values.
%
%	SYNTAX:
%   function [option_out] = get_default_options(option_in,defaults)
%
%	INPUTS:
%		option_in:		Structure containing the user options
%		defaults:		Structure containing the default options to be used
%
%	OUTPUT:
%		option_out:	    Structure containing the function's options
%                       structure with defaults values (where no values were provided)
%
%--------------------------------------------------------------------------

option_out = option_in;

names = fieldnames(defaults);
idx=size(names,1);

% evaluate the user options and defaults and output the refined options
for i=1:idx
	if (isfield(option_out,names{i})==0)
		default_value = getfield(defaults,names{i});
    	option_out = setfield(option_out,names{i},default_value);
	end
end
