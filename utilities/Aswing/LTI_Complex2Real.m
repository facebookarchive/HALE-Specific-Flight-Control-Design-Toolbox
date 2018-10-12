function rsys = LTI_Complex2Real (csys)
% Convert complex-valued LTI continuous objects to real-valued ss objects.

ra = [[real(csys.a) -imag(csys.a)];[imag(csys.a) real(csys.a)]];
rb = [[real(csys.b) -imag(csys.b)];[imag(csys.b) real(csys.b)]];
rc = [[real(csys.c) -imag(csys.c)];[imag(csys.c) real(csys.c)]];
rd = [[real(csys.d) -imag(csys.d)];[imag(csys.d) real(csys.d)]];

rsys = ss(ra,rb,rc,rd);
rsys = rsys(1:size(csys,1),1:size(csys,2));
