function [the] = dhparams(d,a,alph,the)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

DH=[cos(the) -sin(the)*cos(alph) sin(the)*sin(alph) a*cos(the);
    sin(the) cos(the)*cos(alph) -cos(the)*sin(alph) a*sin(the);
    0 sin(alph) cos(alph) d;
    0 0 0 1];
the=[DH(1,4) DH(2,4);DH(3,4) DH(4,4)];
end

