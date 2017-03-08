'''
Compare AlvinXY with Geonav

Should be run with `ipython --pylab`
'''
import numpy as np

from alvinxy import alvinxy as axy
#import alvinxy.alvinxy as axy
reload(axy)

from geonav_transform import geonav_conversions as gc
#import geonav_transform.geonav_conversions as gc
reload(gc)

# Set an origin near NPS.
originlat = 36.59
originlon = -121.89
nx = 10
dx = 100000
ny = 10
dy = 100000
xv, yv = np.meshgrid(np.linspace(0,dx,nx),
                     np.linspace(0,dy,ny))
alatv = np.zeros(yv.shape)
alonv = np.zeros(xv.shape)
glatv = np.zeros(yv.shape)
glonv = np.zeros(xv.shape)

for ii in range(nx):
    for jj in range(ny):
        x = xv[ii,jj]
        y = yv[ii,jj]
        alatv[ii,jj], alonv[ii,jj] = axy.xy2ll(x,y,originlat,originlon)
        glatv[ii,jj], glonv[ii,jj] = gc.xy2ll(x,y,originlat,originlon)

figure(1)
clf()
plot(xv.flatten(),yv.flatten(),'.')
grid(True)


figure(2)
clf()
plot(alonv.flatten(),alatv.flatten(),'r.',label='AlvinXY')
hold(True)
plot(glonv.flatten(),glatv.flatten(),'g.',label='Geonav')
grid(True)
xlabel('Easting [m]')
ylabel('Northing [m]')
title('Comparison of AlvinXY and Geonav for regular spaced X/Y')
legend()

# Now start with lat/lon
nx = 10
dx = 1.0
ny = 10
dy = 1.0
lonv, latv = np.meshgrid(np.linspace(0,dx,nx),
                         np.linspace(0,dy,ny))
axv = np.zeros(lonv.shape)
ayv = np.zeros(latv.shape)
gxv = np.zeros(lonv.shape)
gyv = np.zeros(latv.shape)

for ii in range(nx):
    for jj in range(ny):
        lat = originlat+latv[ii,jj]
        lon = originlon+lonv[ii,jj]
        axv[ii,jj], ayv[ii,jj] = axy.ll2xy(lat,lon,originlat,originlon)
        gxv[ii,jj], gyv[ii,jj] = gc.ll2xy(lat,lon,originlat,originlon)

figure(3)
clf()
plot(lonv.flatten(),latv.flatten(),'.')
grid(True)


figure(4)
clf()
plot(axv.flatten(),ayv.flatten(),'r.',label='AlvinXY')
hold(True)
plot(gxv.flatten(),gyv.flatten(),'g.',label='Geonav')
grid(True)
xlabel('Longitude [deg]')
ylabel('Latitude [m]')
title('Comparison of AlvinXY and Geonav for regular spaced Lat/Lon')
legend()



show()





