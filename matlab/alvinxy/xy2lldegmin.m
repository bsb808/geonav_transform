function [ll] = xy2lldegmin(xy,originll,showit)



if nargin < 3
    showit = 0;
end

[Lat,Lon] = xy2ll(xy(1),xy(2),originll(1),originll(2));
[ll.latdeg,ll.latmin] = deg2degmin(Lat);
[ll.londeg,ll.lonmin] = deg2degmin(Lon);

if showit
    fprintf('X = %6.1f, Y = %6.1f\n',xy(1),xy(2));
    fprintf('Lat = %d , %7.4f N ; %d , %7.4f E \n',...
        ll.latdeg,ll.latmin,ll.londeg,ll.lonmin);
end


