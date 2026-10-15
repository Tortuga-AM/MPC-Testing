# AutoAlign

`AutoAlign` owns autonomous navigation/alignment behavior for the swerve drivetrain.

`Drive` owns hardware I/O, odometry, manual driving, and accepts robot-relative chassis-speed commands.

## Pipeline

```text
Target Pose
  → AD* occupancy-grid planner
  → line-of-sight polyline simplification
  → Bézier smoothing candidates
  → Bézier collision check
  → custom trajectory generation
  → holonomic PID + velocity feedforward
  → swerve setpoint limiting
  → Drive.driveRobotRelative(...)
Basic API
Start alignment
java
Copy code

AutoAlign.getInstance().alignToPose(
    new Pose2d(
        5.0,
        3.0,
        Rotation2d.fromDegrees(180.0)
    )
);
This requests a path from the current estimated robot pose to the target pose.

Cancel alignment
java
Copy code

AutoAlign.getInstance().cancel();
This stops the drivetrain and returns Drive to manual control.

Query status
java
Copy code

boolean aligning = AutoAlign.getInstance().isAligning();
boolean atTarget = AutoAlign.getInstance().isAtTarget();
boolean failed = AutoAlign.getInstance().hasFailed();

String failure = AutoAlign.getInstance().getFailureReason();
AutoAlign States
State	Meaning
IDLE	No alignment request is active.
PLANNING	Waiting for the AD* background thread to publish a route.
FOLLOWING_TRAJECTORY	Following the latest valid generated trajectory.
REPLANNING	Existing route is invalid or robot deviation requires a replacement route.
AT_TARGET	Robot has finished and settled within configured tolerances.
FAILED	No valid route or trajectory could be generated.
CANCELLED	Request was manually cancelled.
Dynamic Obstacles
Dynamic obstacles are rectangles represented by opposite corners.

java
Copy code

List<ADStar.ObstacleRectangle> obstacles = List.of(
    new ADStar.ObstacleRectangle(
        new Translation2d(4.0, 2.0),
        new Translation2d(5.0, 3.0)
    )
);

AutoAlign.getInstance().setDynamicObstacles(obstacles);
Obstacle rectangles should already include inflation for:

robot bumper radius/footprint;
pose-estimation uncertainty;
desired clearance;
opponent movement uncertainty.
Navgrid
The temporary navgrid source is:

text
Copy code

src/main/deploy/pathplanner/navgrid.json
The runtime location after deploy is:

text
Copy code

/home/lvuser/deploy/pathplanner/navgrid.json
The AD* planner currently reads PathPlanner's JSON format:

json
Copy code

{
  "nodeSizeMeters": 0.1,
  "grid": [[false, false], [false, true]],
  "field_size": {
    "x": 16.54,
    "y": 8.02
  }
}
To use a 0.10 meter grid, the file must actually contain:

json
Copy code

"nodeSizeMeters": 0.1
and the correct corresponding grid dimensions.

Key Configuration
All tuning values are in:

text
Copy code

frc.robot.Drive.AutoAlign.AutoAlignConstants
Main values:

Constant	Purpose
MAX_LINEAR_SPEED_METERS_PER_SECOND	Global translational speed limit.
MAX_LINEAR_ACCEL_METERS_PER_SECOND_SQ	Longitudinal trajectory acceleration/deceleration.
MAX_LATERAL_ACCEL_METERS_PER_SECOND_SQ	Curvature-based turn speed constraint.
MAX_ANGULAR_SPEED_RAD_PER_SECOND	Holonomic rotation speed limit.
MAX_MODULE_SPEED_METERS_PER_SECOND	Maximum allowed ideal individual wheel speed.
MAX_MODULE_ACCEL_METERS_PER_SECOND_SQ	Planned/setpoint drive wheel acceleration limit.
MAX_MODULE_STEER_RATE_RAD_PER_SECOND	Planned/setpoint module azimuth turn-rate limit.
REPLAN_CROSS_TRACK_ERROR_METERS	Off-path distance that requests a replan.
BEZIER_HANDLE_SCALES	Bézier smoothing retry scales.
Safety Notes
The navgrid must already be inflated for the robot footprint.
A collision-free AD* polyline does not guarantee a collision-free Bézier curve.
Every Bézier candidate must be collision-checked before following it.
If the robot starts inside an inflated obstacle cell, the planner resolves the requested endpoint to the nearest safe grid cell. This is useful in simulation, but a real robot should use a dedicated low-speed escape behavior.
Trajectories must be constrained for combined translation and rotation module speed, not only chassis speed.
A full replan should start from measured pose and measured velocity, not assume the robot is stopped.