'''
Here is an example of using lat/lon along with AlvinXY coordinates.
'''

import alvinxy
reload(alvinxy)
import numpy as np

# Specify an origin
origin = [21.3190429, -157.6689890]

x = 10
y = 10
lat, lon = alvinxy.xy2ll(x,y,origin[0],origin[1])
print ('lat: %.10f, lon:%.10f'%(lat,lon))
xx,yy = alvinxy.ll2xy(lat,lon,origin[0],origin[1])
print ('x: %.10f, y:%.10f'%(xx,yy))

# Use vectorized version
x = np.arange(1,100)
y = np.arange(1,100)
lat,lon = alvinxy.vxy2ll(x,y,origin[0],origin[1])
xx,yy = alvinxy.vll2xy(lat,lon,origin[0],origin[1])
print xx
print yy
