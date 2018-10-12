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
% Developer: David Flamholz
% specify a tolerance below which an individual element of a matrix would 
% be considered zero.
function [PbhStruct]=PbhTest(A,B,C,tol)
PbhStruct=struct;
if nargin<4
    tol=sqrt(eps);
end
[V,D,W] = eig(A);
% rank(V)
% rank(W)
n=size(A,1); % number of states
m=size(B,2); % number of inputs
p=size(C,1); % number of outputs

Ctrl_check_matrix=W'*B; % size nxm
% Check no rows in W' are orthogonal to columns of B, that is, no row of
% Ctrl_check_matrix is all zeros
Obs_check_matrix=C*V; % size pxn
% Check no columns of V are orthogonal to rows of C, that is, no column of
% Obs_check_matrix is all zeros
UC_space=[];
C_space=[];
O_space=[];
UO_space=[];
for i=1:n
    % checking for null rows of Ctrl_check_matrix
    if norm(Ctrl_check_matrix(i,:))<=(tol*sqrt(m)) && abs(max(Ctrl_check_matrix(i,:)))<tol
        UC_space=[UC_space W(:,i)];
    else
        C_space=[C_space W(:,i)];
    end
    % checking for null columns of Obs_check_matrix
    if norm(Obs_check_matrix(:,i))<=(tol*sqrt(p))
        UO_space=[UO_space V(:,i)];
    else
        O_space=[O_space V(:,i)];
    end    
end


% if A is not diagonalizable, then we do not have a complete set of left
% and right eigenvectors and we need to check if the nullspaces of W' and
% V' are controllable and observable
Left_eig_null=null(W');
Right_eig_null=null(V');
P=ctrb(A,B);
Q=obsv(A,C);

for j_l=1:size(Left_eig_null,2)
    w=Left_eig_null(:,j_l);
    if norm(w'*P)<=(tol*sqrt(n*m))
        UC_space=[UC_space w];
    else
        C_space=[C_space w];
    end
end

for j_r=1:size(Right_eig_null,2)
    v=Right_eig_null(:,j_r);
    if norm(Q*v)<=(tol*sqrt(n*p))
        UO_space=[UO_space v];
    else
        O_space=[O_space v];
    end
end

% Back to main code body


C_UO_space=[];
O_UC_space=[];
%OC_space=C_space; % initializing this to either one of the controllable or observable subspaces
n_cntr=size(C_space,2);
n_obs=size(O_space,2);
OC_index_list=1:n_cntr; % initializing this to either the dimension one of the controllable or the observable subspaces


for j_o=1:n_obs
    v_obs=O_space(:,j_o);
    if norm(v_obs'*C_space)<=(tol*sqrt(n_cntr))
       O_UC_space=[O_UC_space v_obs];
    end
end

for j_c=1:n_cntr
    v_ctr=C_space(:,j_c);
    if norm(v_ctr'*O_space)<=(tol*sqrt(n_obs))
        C_UO_space=[C_UO_space v_ctr];
        OC_index_list=OC_index_list(find(OC_index_list~=j_c));
    end
end

OC_space=C_space(:,OC_index_list);

% n_min=min(n_cntr,n_obs);
% if n_min==n_cntr
%     Min_Mat=C_space;
%     Max_Mat=O_space;
%     n_max=n_obs;
% else
%     Min_Mat=O_space;
%     Max_Mat=C_space;
%     n_max=n_cntr;
% end
% 
% for j=1:n_min
%     v=Min_Mat(:,j);
%     if norm(v'*Max_Mat)<=(tol*sqrt(n_max))
        

PbhStruct.ObservableSubspace=O_space;
PbhStruct.UnobservableSubspace=UO_space;
PbhStruct.ControllableSubspace=C_space;
PbhStruct.UncontrollableSubspace=UC_space;
PbhStruct.CO_Subspace=OC_space;
PbhStruct.C_notO_Subspace=C_UO_space;
PbhStruct.O_notC_Subspace=O_UC_space;
PbhStruct.ControllableDimension=size(C_space,2);
PbhStruct.ObservableDimension=size(O_space,2);
PbhStruct.CO_Dimension=size(OC_space,2);
end
        
