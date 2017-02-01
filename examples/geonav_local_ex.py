import geonav_transform.geonav_conversions as gc
reload(gc)

## geonav conversions
# Define a local orgin
olat = 37.
olon = -122.


# Convert a lat/lon to a local x/y
lat = olat
lon = olon
outmy, outmx, outmzone = gc.LLtoUTM(olat,olon)
utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
print('Lat: %.4f, Lon:%.4f >> Y: %.1f, X: %.1f, Z:%s'%(lat,lon,utmy,utmx,utmzone))
y = utmy-outmy
x = utmx-outmx

xx,yy = gc.ll2xy(lat,lon,olat,olon)

# Back to lat/lon
llat, llon = gc.UTMtoLL(utmy,utmx,outmzone)
print(' delta lat = %.10f, delta lon = %.10f'%(llat-lat,llon-lon))
