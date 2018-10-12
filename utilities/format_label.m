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


function label_formatted = format_label(label)
    % replace greek symbols with the Matlab text formatting.
    replace_strings = {'alpha', 'beta', 'gamma', 'phi', 'delta', 'psi', ...
                       'theta', 'tau', 'omega'};
    for i = 1:length(replace_strings)
        replace_string = replace_strings{i};
        if strfind(label, replace_string)
            label = strrep(label, replace_string, ...
                            strcat('\', replace_string));
        end
    end
    if strfind(label, '_dot')
        label = regexprep(label, '(.*)_dot(.*)', '\\dot{$1}$2', 'once');
    elseif strfind(label, '_ddot')
        label = regexprep(label, '(.*)_ddot(.*)', '\\ddot{$1}$2', 'once');
    end
    
    if strfind(label, '_')
        label = regexprep(label, '_(.*)', '_{$1}');
        i_ = strfind(label, '_');
        label(i_(2:end)) = ',';
    end
    label_formatted = label;
end