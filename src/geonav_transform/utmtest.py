import random
from math import *
import geonav_conversions as nc
reload(nc)

lat = 36.6
lon = -121.9

utmx, utmy, utmzone = nc.LLtoUTM(lat,lon)
print ("UTM X: %.10f, Y: %.10f, Zone: %s"%(utmx,utmy,utmzone))

lat, lon = nc.UTMtoLL(utmx,utmy,utmzone)
print("Lat: %.10f, Long: %.10f"%(lat,lon))

cnt = 1
err = []
for ii in range(10000000):
    lat = random.uniform(-80.0,80.0)
    lon = random.uniform(-180.0,179.9999)
    
    utmx, utmy, utmzone = nc.LLtoUTM(lat,lon)
    lat2, lon2 = nc.UTMtoLL(utmx,utmy,utmzone)

    utmx2, utmy2, utmzone2 = nc.LLtoUTM(lat2,lon2)
    dlat = lat-lat2
    dlon = lon-lon2
    tol = 1e-6
    if (abs(dlat)>tol) or (abs(dlon)>tol):
        print("-------%d-------"%cnt)
        print("Lat: %.10f, Long: %.10f"%(lat,lon))
        print ("UTM X: %.10f, Y: %.10f, Zone: %s"%(utmx,utmy,utmzone))
        print("dLat: %.10f, dLong: %.10f"%(dlat,dlon))
        dutmx = utmx-utmx2
        dutmy = utmy-utmy2
        print("dUtmx: %.10f, dUtmy: %.10f"%(dutmx,dutmy))
        cnt +=1
        err.append(sqrt(dutmx**2+dutmy**2))

print err
print("Max Error [m]: %.10f"%max(err))
    #if (ii%1000)==0:
    #    print("ii: %d"%ii)
    
