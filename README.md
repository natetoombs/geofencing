# ROScopter-compatible Geofencing Package
This package adds a minimal geofencing capability to ROScopter, allowing the user to define 
a flyable area and choose a failsafe behavior if the area is exited.

## Usage
### Parameters
The initial geofence is set from parameters in the ROScopter .yaml file.

`geofence_points` is a list of either NE or LatLon coordinates used to define the fence.

`gps_points` is used to determine if the above list uses NE points (meters) or LatLon coordinates (decimal). If `True`, then the points will be converted to points in the NED frame.

`land_delay` is the amount of time (seconds) before switching from a first attempted failsafe behavior to a second. For example, the first failsafe behavior could be to attempt to return to base when the drone flies outside the fence. If the drone remains outside the fence, a second failsafe behavior is activated, which could be to simply land in place.

### Services

`add_geofence_point` adds a point to the geofence. Uses 2 arguments: position in N, position in E.

`clear_geofence` clears all geofence points.
