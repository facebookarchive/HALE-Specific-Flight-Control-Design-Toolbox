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


% Accepts two number a and b as inputs and outputs
% the Close_enough_boolean flag if they are in the vicinity of each other
% w.r.t the percent_tolerance,
function [Close_enough_boolean]=CloseEnough(a,b,percent_tolerance)
    if a<1e3
        a=0;
    end
    if b<1e3
        b=0;
    end
    if nargin<3
        percent_tolerance=0.0001;
    end

    if (isnan(a) && isnan(b))
        Close_enough_boolean=true;
    elseif (isinf(a) && isinf(b))
        Close_enough_boolean=true;
    else
        if a~=0
            Close_enough_boolean=((abs(a-b)/a)<=percent_tolerance);
        elseif b~=0
            Close_enough_boolean=((abs(a-b)/b)<=percent_tolerance);
        else
            Close_enough_boolean=true;
        end
    end

end
