package frc.controls.paths;

import frc.controls.CubicSplineFollower.Waypoint;
import frc.util.Pose;
import frc.util.terrorMath;

public class paths {

    public interface PATH_SET{
		Pose START = null;
		Waypoint[] WAYPOINTS = null;
    }
    
    public static class MADNESS implements PATH_SET{
        public final static Pose START = new Pose(terrorMath.toMeters(4.5),terrorMath.toMeters(-13.5),180);
        public static final Waypoint[] WAYPOINTS = {new Waypoint(terrorMath.toMeters(31.5), terrorMath.toMeters(-12.5), 90, -0.3, false)};
    }
}
