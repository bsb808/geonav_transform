'''
Here is an example of using lat/lon along with AlvinXY coordinates.

HISTORY
2015.07.30  bsb  For the Makai UPSV tests
'''

import alvinxy

# Specify an origin
org = [21.3190429, -157.6689890]

# LatLon --> XY
llpnts = [[21.319496, -157.669128],
          [21.319856, -157.668770],
          [21.319493, -157.668305]]

# Convert them one at a time to XY
xypnts = alvinxy.ll2xyLL(llpnts,org)

# Report the results
for ll,xy in zip(llpnts,xypnts):
    print ("%s --> %s"%(str(ll),str(xy)))

# Now convert back to lat/lon
ll2pnts = alvinxy.xy2llLL(xypnts,org)

# Report the results
for ll,ll2 in zip(llpnts,ll2pnts):
    print ("%s --> %s"%(str(ll),str(ll2)))
