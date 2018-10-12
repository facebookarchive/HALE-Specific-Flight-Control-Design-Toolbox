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

% This Function assumes InputFile has four 4 colunms that may not be
% uniform in size (number of rows per column). For example, there can be 4
% alts, 2 airspeeds, 1 gamma, and 1 phi and that would correspond to a
% column with 4 row elements, followed by a column with two row elements,
% followed by 1 and 1
function [alts eass gammas phis]=getFltCndInput(InputFile) 

t=readtable(InputFile);
headers=fieldnames(t);

alts=getfield(t,char(headers(1)));
eass=getfield(t,char(headers(2)));
gammas=getfield(t,char(headers(3)));
phis=getfield(t,char(headers(4)));

alts=(alts(find(isfinite(alts))))';
eass=(eass(find(isfinite(eass))))';
gammas=(gammas(find(isfinite(gammas))))';
phis=(phis(find(isfinite(phis))))';

end
