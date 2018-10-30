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
function [] = prettyPlots()
%PRETTYPLOTS improves plot visualization for presentation

    %% Get all the line handles from root and set its width
    allLineHandles = findall(groot, 'Type', 'line');
    set(allLineHandles, 'LineWidth', 2.0);

    %% Get all axes handles and set its color
    allAxesHandles = findall(groot, 'Type', 'Axes');
    set(allAxesHandles, 'FontName','Arial','FontWeight','Bold','LineWidth',3,...
        'FontSize',12);

    %% Get titles
    alltext = findall(groot, 'Type', 'Text');
    set(alltext,'FontName','Arial','FontWeight','Bold','FontSize',12, 'Interpreter', 'None');
end

