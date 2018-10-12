function sys = ParseROM (NumInputs, NumOutputs, case_name)
%% This function parses the ROM output from aswing into A,B,C and D matrices for Matlab
% The output is a state space system

% Construct A matrix
fid = fopen(strcat(case_name,'.e00'));
R = textscan(fid,'%f %f %f','HeaderLines',3,'Delimiter','\t');
eigs = conj_sort(R{2},R{3},'row');
A = diag(eigs);
[~, idx] = unique(diag(A));
A = A(idx,idx);

% Construct B,C,D matrices 
NumModes = length(unique(real(eigs)));
fid = fopen(strcat(case_name,'.rom'));
B=[];
C=[];
D = [];
for j = 1:NumModes
R = textscan(fid,'%f %f %s',NumInputs,'CommentStyle','#','Delimiter','\t');
B = [B;conj_sort(R{1},R{2},'col').'];
R = textscan(fid,'%f %f %s',NumOutputs,'CommentStyle','#','Delimiter','\t');
C = [C,conj_sort(R{1},R{2},'col')];
end
B = B(idx, :);
C = C(:, idx);
for j = 1:NumInputs
R = textscan(fid,'%f %f %s',NumOutputs,'CommentStyle','#','Delimiter','\t');
D = [D,[R{1}+R{2}*i]];
end


% Construct state-space object
sys_complex = ss(A,B,C,D);
% Aswing outputs a complex valued state space, but Simulink only accepts
% real valued systems.  Compute equivalent real valued system.
sys = LTI_Complex2Real(sys_complex);

% The complex to real function adds many duplicate states,
% remove them with balred.
sys = balred(sys,numel(diag(A)));  
end

function vec = conj_sort(x,y,method)
%x: real part
%y: imaginary part 
len = length(x);
switch method
    case 'row' 
    vec = zeros(len*2,1);
    vec(1:2:end) = complex(x,y);
    vec(2:2:end) = conj(complex(x,y));
    
    case 'col'
    vec = zeros(len,2);    
    vec(:,1) = complex(x,y);
    vec(:,2) = conj(complex(x,y));  
end
end