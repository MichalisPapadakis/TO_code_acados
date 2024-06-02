function [xlin] = lin_interpolate_traj(x0,xref,N)
%Linearly interpolates from x0 to xref. 

ns = length(xref);
xlin = zeros(ns,N);

for iS = 1:ns
    xlin(iS,:) = linspace(x0(iS),xref(iS),N);
end

for i=1:N
    x_ = DK(xlin(1:5,i));
    xlin(1:5,i) = x_;
end

xlin = reshape(xlin,N*ns,1);