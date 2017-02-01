'''
Illustrative example of UTM and local xy coordinates

Should be run with ipython --pylab
'''

import geonav_transform.geonav_conversions as gc
reload(gc)
import alvinxy.alvinxy as axy
reload(axy)

# Consider Zone 10S
# Longitude from -120 to -126
# Latitude from 32 to 40 

Lon = linspace(-126.0,-120.0,100)
Lat = linspace(32.0,40.0,100)

# Define local origin at lower right of UTM zone
olon = -126.0
olat = 32.0

Xutm = []
Yutm = []
Xxy = []
Yxy = []
Xa = []
Ya = []
for lat in Lat:
    lon = -126.0
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    x, y = gc.ll2xy(lat,lon,olat,olon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)
    print utmzone
    Xutm.append(utmx)
    Yutm.append(utmy)
    Xxy.append(x)
    Yxy.append(y)
    Xa.append(xa)
    Ya.append(ya)

Xutm = array(Xutm)
Yutm = array(Yutm)
figure(1)
clf()
subplot(211)
plot(Lat,Xutm,'.')
title('Lon = %.2f'%lon)
ylabel('UTM X (Easting) [m]')
grid(True)
subplot(212)
plot(Lat,Yutm,'.')
ylabel('UTM Y (Northing) [m]')
xlabel('Lat [deg]')
grid(True)

figure(2)
clf()
subplot(211)
plot(Lat,Xutm-Xutm[0],'.')
title('Lon = %.2f'%lon)
ylabel('Delta UTM X (Easting) [m]')
grid(True)
subplot(212)
plot(Lat,Yutm-Yutm[0],'.')
ylabel('Delta UTM Y (Northing) [m]')
xlabel('Lat [deg]')
grid(True)

figure(3)
clf()
subplot(211)
plot(Lat,Xxy,'.',label='UTM local')
hold(True)
plot(Lat,Xa,'r.',label='Alvin XY')
title('Lon = %.2f, Origin: Lat=%.2f, Lon=%.2f'%(lon,olat,olon))
legend()
ylabel('Local X (Easting) [m]')
grid(True)
subplot(212)
plot(Lat,Yxy,'.')
hold(True)
plot(Lat,Ya,'r.')
ylabel('Local Y (Northing) [m]')
xlabel('Lat [deg]')
grid(True)


Xutm = []
Yutm = []
Xxy = []
Yxy = []
Xa = []
Ya = []
for lon in Lon:
    lat = olat
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    x, y = gc.ll2xy(lat,lon,olat,olon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)
    print utmzone
    Xutm.append(utmx)
    Yutm.append(utmy)
    Xxy.append(x)
    Yxy.append(y)
    Xa.append(xa)
    Ya.append(ya)

Xutm = array(Xutm)
Yutm = array(Yutm)
figure(4)
clf()
subplot(211)
plot(Lon,Xutm,'.')
title('Lat = %.2f'%lat)
ylabel('UTM X (Easting) [m]')
grid(True)
subplot(212)
plot(Lon,Yutm,'.')
ylabel('UTM Y (Northing) [m]')
xlabel('Lon [deg]')
grid(True)

figure(5)
clf()
subplot(211)
plot(Lon,Xutm-Xutm[0],'.')
title('Lat = %.2f'%lat)
ylabel('Delta UTM X (Easting) [m]')
grid(True)
subplot(212)
plot(Lon,Yutm-Yutm[0],'.')
ylabel('Delta UTM Y (Northing) [m]')
xlabel('Lon [deg]')
grid(True)

figure(6)
clf()
subplot(211)
plot(Lat,Xxy,'.',label='UTM local')
hold(True)
plot(Lat,Xa,'r.',label='Alvin XY')
title('Lat = %.2f, Origin: Lat=%.2f, Lon=%.2f'%(lat,olat,olon))
ylabel('Local X (Easting) [m]')
grid(True)
subplot(212)
plot(Lat,Yxy,'.')
hold(True)
plot(Lat,Ya,'r.')
ylabel('Local Y (Northing) [m]')
xlabel('Lon [deg]')
grid(True)

show()
