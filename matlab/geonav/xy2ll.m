% [lat,lon] = xy2ll(x, y, orglat, orglon)
%
% XY2LL: Geonav
% Converts local x/y to Lat/Lon using UTM coordinates
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
% lat - Latitude in decimal degrees
% lon - Longitude in decimal degrees
%
% EXAMPLE
% [lat,lon] = ll2xy(878.0, 1118.8, 37, -122);
%