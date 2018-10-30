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
%   Developer:  Abe Martin, Hamid Bolandhemmat
%
function rsys = LTI_Complex2Real (csys)
% Convert complex-valued LTI continuous objects to real-valued ss objects.

ra = [[real(csys.a) -imag(csys.a)];[imag(csys.a) real(csys.a)]];
rb = [[real(csys.b) -imag(csys.b)];[imag(csys.b) real(csys.b)]];
rc = [[real(csys.c) -imag(csys.c)];[imag(csys.c) real(csys.c)]];
rd = [[real(csys.d) -imag(csys.d)];[imag(csys.d) real(csys.d)]];

rsys = ss(ra,rb,rc,rd);
rsys = rsys(1:size(csys,1),1:size(csys,2));
