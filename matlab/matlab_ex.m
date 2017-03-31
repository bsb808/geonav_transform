% Example of Global <-> Local coordinate transforms using
% either geonav or alvinxy
%
% Note - since the geonav functions are MEX functions, you 
% will need to run the make_mex.sh script to create the MEX
% files for your particular MATLAB installation.

% Specify paths to the two toolboxes
alvinpath = './alvinxy';
geonavpath = './geonav';

% Make sure to remove both paths so that only one is active at a time
rmpath(alvinpath);
rmpath(geonavpath);

% Select which toolbox to use by commenting out one of the following 
addpath(geonavpath); disp('**Using Geonav**');
%addpath(alvinpath); disp('**Using AlvinXY**');

% Define a local orgin, latitude and longitude in decimal degrees
olat = 37.0;
olon = -122.0;

% Pick a point not too far from the origin
lat = olat+0.01;
lon = olon+0.01;


% Convert to X/Y and back
% Convert a lat/lon to a local x/y
fprintf('Convert from lat/lon to x/y\n')
[x,y] = ll2xy(lat,lon,olat,olon);
fprintf('ll2xy, Lat: %.4f, Lon:%.4f >> X: %.1f, Y: %.1f\n',lat,lon,x,y);
% Back to lat/lon
[nlat, nlon] = xy2ll(x,y,olat,olon);
fprintf('xy2ll, X: %.1f, Y: %.1f >> Lat: %.4f, Lon:%.4f\n',x,y,nlat,nlon);
fprintf('\t Delta, Lat: %.12f, Lon: %.12f [deg]\n',lat-nlat,lon-nlon);

