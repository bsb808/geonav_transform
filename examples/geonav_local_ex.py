#!/usr/bin/env python 
'''
Example of using geonav_transform and alvinxy
'''
import geonav_transform.geonav_conversions as gc
reload(gc)
import alvinxy.alvinxy as axy
reload(axy)


# Define a local orgin
olat = 37.
olon = -122.

# Pick a point not too far from the origin
lat = olat+0.01
lon = olon+0.01

# Convert to UTM
outmy, outmx, outmzone = gc.LLtoUTM(olat,olon)
utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
print('Using geonav_tranform')
print('Convert to UTM, Lat: %.4f, Lon:%.4f >> UTMY: %.1f, UTMX: %.1f, Zone:%s'%(lat,lon,utmy,utmx,utmzone))

latt, lonn = gc.UTMtoLL(utmy,utmx,utmzone)
print('Convert back to Lat/Lon, Lat: %.4f, Lon: %.4f'%(latt,lonn))
print('Delta, Lat: %.12f, Lon: %.12f [deg]'%(lat-latt,lon-lonn))

y = utmy-outmy
x = utmx-outmx

xx,yy = gc.ll2xy(lat,lon,olat,olon)

# Back to lat/lon
llat, llon = gc.UTMtoLL(utmy,utmx,outmzone)
print(' delta lat = %.10f, delta lon = %.10f'%(llat-lat,llon-lon))
