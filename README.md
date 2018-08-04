# Vehicle-Path-Planning-Project
   
### Available Data.
The data found in highway_map.csv gives us the location of known wayopints along our path. These waypoints are provided in Frenet cordinates which makes it a lot easier to work with.

We also have sensor data provided by the simulator with information about the current position of all the vehicles around us. In practice, this information will be derived from the Lidar, Radar and camera sensors of our vehicle.

### Goals
In this project the goal was to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We wanted our car to try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars try to change their lanes too. We wanted our car to safely and legaly navigate the track with comfort of the car occupants in mind. We therefore had to ensure we sped up, slowed down and changed lanes without any jerk.

### Path Points
The simulator controls the vehicle by navigating to all the path points we provide. The simulator processes each path point every 20ms.
Our main object was to therefore provide the right path points depending on our situation, hence "Path Planning".

### Modeling the Path
To generate the right path points we do the following:

#### State Machine
1. Check our proximity to all cars around us.
2. Maintain an "is lane obstructed" state for our lane and each lane to the left and right of our car.
3. If our lane is obstructed slow down.
4. If our lane is obstructed but other lanes are free, swithc lanes.
5. If there's no obstruction, speed up and cap at the speed limit.

#### Generate New Path
1. Add the last two unprocessed points from the previous path to a temporary new path. Also get the heading direction of our car.
2. Use the reported waypoints from the map to exptrapolate to artificial waypoints 30, 60 and 90 meters ahead.
3. Add the atrificial waypoints to our temporary new path.
4. Shift the points in our temporary path to vehicle coordinates and create a Spline function from them. The Spline function will give us a smooth curve that links all the temporary points.
5. We add all the previously unprocessed path points to our new list of path points in order to aid a smooth transition.
6. Based on our target velocity and path horizon, we determine how far apart our newly added points should be from each other. This directly influences the speed of the vehicle.
7. With the knowledge of how far apart our new points should be, we use our spline function to generate enough new path points until we have as many as we need.
8. We pass all the previously unprocessed and newly create path points to the simulator for it to process.

### Demo
#### Click to see full video of sample run

[![Autonomous Vehicle Path Planning](./output/autonomous_path_planning.gif)](http://www.youtube.com/watch?v=T-D1KVIuvjA)

