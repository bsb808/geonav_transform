import geonav_transform.geonav_conversions as gc
reload(gc)
import alvinxy.alvinxy as axy
reload(axy)


## geonav conversions
# Define a local orgin
olat = 36.6
olon = -121.9

# for zone 10S

dlat = 0.0
dlon = 0.01
Xg = []
Yg = []
Dg = []
dX = []
dY = []
dD = []

for n in range(100):
    lat = olat + n*dlat
    lon = olon + n*dlon

    xx,yy = gc.ll2xy(lat,lon,olat,olon)
    xxx, yyy = axy.ll2xy(lat,lon,olat,olon)
    Xg.append(xx)
    Yg.append(yy)
    Dg.append(sqrt(xx**2+yy**2))
    dX.append(xx-xxx)
    dY.append(yy-yyy)
    dD.append(sqrt( (xx-xxx)**2+(yy-yyy)**2))

figure(1)
clf()
subplot(211)
plot(Xg,dX,'.')
subplot(212)
plot(Xg,dY,'.')
show()
