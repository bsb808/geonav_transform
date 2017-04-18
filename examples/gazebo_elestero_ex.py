#!/usr/bin/env python 
'''
In this example we use the geonav coordinate system to move between three 
coordinate systems
1) Lat/lon
2) Local geonav coordinates
3) Gazebo local coordinates

'''

# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
reload(gc)

# Define a local orgin, latitude and longitude in decimal degrees
# This is the standard geonav origin for working at El Estero Lake
olat = 36.595
olon = -121.89

# This is somewhat confusing, but Gazebo also has a local coordinate system.
# In the elestero.launch file we define the gazebo origin relative to the 
# heightmap used to represent the lake.
gaz_lat = 36.596524
gaz_lon = -121.888169

# So in geonav coordinates, the Gazebo origin is...
xg, yg = gc.ll2xy(gaz_lat,gaz_lon,olat,olon)
print('Geonav ll2xy, Lat: %.4f, Lon:%.4f >> X: %.1f, Y: %.1f'
      %(gaz_lat,gaz_lon,xg,yg))
# This corresponds to where the vehicle is when we spawn it at 0,0 in Gazebo

# If we know the target location in the geonav coordinates
x = 137.6
y = 54.0
# Then the location in Gazebo coordinates is...
print('Target Gazebo coordinates, X: %.3f, Y: %.3f [m]'%(x-xg,y-yg))

# Similarly for the repulsive field location
x = 138.2
y = 75.9
# Then the location in Gazebo coordinates is...
print('Target Gazebo coordinates, X: %.3f, Y: %.3f [m]'%(x-xg,y-yg))


