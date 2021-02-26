package frc.controls.paths;

import frc.controls.CubicSplineFollower.Waypoint;
import frc.util.Pose;

public class paths {
    public interface PATH_SET{
		Pose START = null;
		Waypoint[] WAYPOINTS = null;
	}
}
