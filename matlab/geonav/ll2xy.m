% [x,y] = ll2xy(lat, lon, orglat, orglon)
%
% LL2XY: Geonav
% Converts Lat/Lon to a local x/y using UTM coordinates
% The actual function is a MEX function based on the .cpp file
% for compatibility with ROS geonav_transform package.  This 
% file is just the help documentation
%
% INPUT
% x - Easting in m (Geonav local grid)
% y - Northing in m (Geonav local grid)
% orglat - origin location latitude in decimal degrees
% orglon - origin location longitude in decimal degrees
% 
% OUTPUT
% x - Easting in m (Geonav local grid)XS
% y - Northing in m (Geonav local grid)
%
% HISTORY
%
% EXAMPLE
% [x,y] = ll2xy(37.01,-121.99,36,-122);
%
