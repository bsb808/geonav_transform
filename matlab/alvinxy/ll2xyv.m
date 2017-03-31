function [x,y] = (ll,originll)
% given the lat and lon in decimal degrees along with the orgin (also
% in decimal degrees, returns the x,y using a mercator
% projection
% History
% 02.07.2006 bbing using DSL's version as a base and creating a similar
% script 
% 
% Ex
% [xy] = ll2xy

% ll2xy(degmin2deg(latdeg,latmin),degmin2deg(londeg,lonmin),...
%    originlat,originlon);

lat = ll(:,1));
lon = ll(:,2);
orglat = originll(1);
orglon = originll(2);

x = (lon - orglon) * mdeglon(orglat);
y = (lat - orglat) * mdeglat(orglat);
