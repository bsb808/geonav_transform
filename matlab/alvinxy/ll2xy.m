function [x,y] = ll2xy(lat, lon, orglat, orglon)

% [x,y] = ll2xy(lat, lon, orglat, orglon)
%
% LL2XY: AlvinXY
% Converts Lat/Lon (WGS84) to Alvin XY's using a Mercator projection.
%
% INPUT
% x - Easting in m (Alvin local grid)
% y - Northing in m (Alvin local grid)
% orglat - origin location latitude in decimal degrees
% orglon - origin location longitude in decimal degrees
% 
% OUTPUT
% x - Easting in m (Alvin local grid)
% y - Northing in m (Alvin local grid)
%
% HISTORY
%
% EXAMPLE
% [x,y] =
% ll2xy(degmin2deg(latdeg,latmin),degmin2deg(londeg,lonmin),...
%    originlat,originlon);
%
% SEE ALSO
% xy2ll.m
% mdeglon.m and mdeglat.m

x = (lon - orglon) * mdeglon(orglat);
y = (lat - orglat) * mdeglat(orglat);
