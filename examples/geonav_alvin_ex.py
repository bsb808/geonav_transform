#!/usr/bin/env python 
'''
Example illustrating use of local coordinate systems.
'''

# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
reload(gc)
# Import AlvinXY transformation module
import alvinxy.alvinxy as axy
reload(axy)

# Define a local orgin, latitude and longitude in decimal degrees
olat = 37.
olon = -122.

# Pick a point not too far from the origin
lat = olat+0.01
lon = olon+0.01

# Convert to UTM
outmy, outmx, outmzone = gc.LLtoUTM(olat,olon)
utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
print('UTM Conversion Using geonav_tranform')
print('Convert to UTM, Lat: %.4f, Lon:%.4f >> UTMY: %.1f, UTMX: %.1f, Zone:%s'%(lat,lon,utmy,utmx,utmzone))

latt, lonn = gc.UTMtoLL(utmy,utmx,utmzone)
print('Convert back to Lat/Lon, Lat: %.4f, Lon: %.4f'%(latt,lonn))
print('Delta, Lat: %.12f, Lon: %.12f [deg]'%(lat-latt,lon-lonn))
print(' ')

# Convert to X/Y and back
# Convert a lat/lon to a local x/y
print('Convert from lat/lon to x/y')
xg, yg = gc.ll2xy(lat,lon,olat,olon)
xa, ya = axy.ll2xy(lat,lon,olat,olon)
print('Geonav ll2xy, Lat: %.4f, Lon:%.4f >> X: %.1f, Y: %.1f'
      %(lat,lon,xg,yg))
print('AlvinXY ll2xy, Lat: %.4f, Lon:%.4f >> X: %.1f, Y: %.1f'
      %(lat,lon,xa,ya))
# Back to lat/lon
glat, glon = gc.xy2ll(xg,yg,olat,olon)
alat, alon = axy.xy2ll(xg,yg,olat,olon)
print('Geonav xy2xy, X: %.1f, Y: %.1f >> Lat: %.4f, Lon:%.4f '
      %(xg,yg,glat,glon))
print('\t Delta, Lat: %.12f, Lon: %.12f [deg]'
      %(lat-glat,lon-glon))
print('AlvinXY xy2xy, X: %.1f, Y: %.1f >> Lat: %.4f, Lon:%.4f '
      %(xa,ya,alat,alon))
print('\t Delta, Lat: %.12f, Lon: %.12f [deg]'
      %(lat-alat,lon-alon))

