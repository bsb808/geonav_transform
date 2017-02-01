#!/usr/bin/env python
'''
Copyright (C) 2015 Brian S. Bingham

This file is part of the FRL Vehcile Software (FVS) system.  FVS is proprietary and confidential.  No part of FVS may be disclosed, disseminated, distributed or otherwise conveyed in any matter to a third party without the prior written consent of Brian Bigham
'''
'''
alvinxy module

'''
from math import *

def  mdeglat(lat):
    '''
    % dy = mdeglat(lat)
    %
    % MDEGLAT- used in converting lat/lon <-> xy
    % Meters per degree Latitude at the given Latitude
    %
    % INPUT
    % lon - longitude in decimal degrees
    %
    % OUTPUT
    % dx - meters per degree latitude
    %
    % SEE ALSO
    % xy2ll.m and ll2xy.m
    % mdeglon.m and mdeglat.m
    '''
    latrad = lat*2*pi/360 ;

    dy = 111132.09 - 566.05 * cos(2.0*latrad) \
         + 1.20 * cos(4.0*latrad) \
         - 0.002 * cos(6.0*latrad)
    return dy

def mdeglon(lat):
    '''
    % dx = mdeglon(lat)
    %
    % MDEGLON - used in converting lat/lon <-> xy
    % Meters per degree Longitude at the given Latitude
    %
    % INPUT
    % lat - latitude in decimal degrees
    %
    % OUTPUT
    % dx - meters per degree longitude
    %
    % SEE ALSO
    % xy2ll.m and ll2xy.m
    % mdeglon.m and mdeglat.m
    '''
    latrad = lat*2*pi/360 
    dx = 111415.13 * cos(latrad) \
         - 94.55 * cos(3.0*latrad) \
	+ 0.12 * cos(5.0*latrad)
    return dx

def xy2ll(x, y, orglat, orglon):

    '''
    XY2LL
    % Converts Alvin XYs to Lat/Lon (WGS84) using a Mercator projection.
    %
    % INPUT
    % x - Easting in m (Alvin local grid)
    % y - Northing in m (Alvin local grid)
    % orglat - origin location latitude in decimal degrees
    % orglon - origin location longitude in decimal degrees
    % 
    % HISTORY
    %
    %
    % SEE ALSO
    % ll2xy.m
    % mdeglon.m and mdeglat.m
    %
    '''
    lon = x/mdeglon(orglat) + orglon
    lat = y/mdeglat(orglat) + orglat

    return [lat, lon]

def ll2xy(lat, lon, orglat, orglon):
    '''
    % [x,y] = ll2xy(lat, lon, orglat, orglon)
    %
    % LL2XY
    % Converts Lat/Lon (WGS84) to Alvin XYs using a Mercator projection.
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
    '''

    x = (lon - orglon) * mdeglon(orglat);
    y = (lat - orglat) * mdeglat(orglat);
    return [x,y]


def ll2xyL(latlon,origin):
    ''' for lists of coordintes (length 2)'''
    return ll2xy(latlon[0],latlon[1],origin[0],origin[1])

def cc2xyL(lonlat,origin):
    ''' google earth uses "coordinates" which are lon-lat (reversed order)
    '''
    return ll2xy(lonlat[1],lonlat[0],origin[0],origin[1])

def ll2xyLL(latlon,origin):
    ''' for lists of lists '''
    xy = []
    for ll in latlon:
        xy.append(ll2xyL(ll,origin))
    return xy

def cc2xyLL(lonlat,origin):
    ''' for lists of lists '''
    xy = []
    for ll in lonlat:
        xy.append(cc2xyL(ll,origin))
    return xy

def xy2llL(xy,origin):
    return xy2ll(xy[0],xy[1],origin[0],origin[1])

def xy2llLL(xy,origin):
    ll = []
    for x in xy:
        ll.append(xy2llL(x,origin))
    return ll
                 
    
def xy2ccL(xy,origin):
    ''' google earth uses "coordinates" which are lon-lat (reversed order)
    '''
    #return (xy2llL(xy,origin)).reverse()
    ll = xy2llL(xy,origin)
    ll.reverse()
    return ll

def xy2ccLL(XY,origin):
    ''' for lists of lists '''
    coordinates = []
    for xy in XY:
        coordinates.append(xy2ccL(xy,origin))
    return coordinates
	
