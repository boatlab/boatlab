function F = psiPSD( lambda, xdata )
%SCRIPTFUNC Summary of this function goes here
%   Detailed explanation goes here
F = ( 2*lambda*w_0*sigma * ( xdata / (sqrt( (w_0^2 - xdata^2)^2 + (2*lambda*w_0*xdata)^2) ) ) )^2
end
