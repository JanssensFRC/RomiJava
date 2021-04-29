package frc.controls.paths;

import frc.controls.CubicSplineFollower.Waypoint;
import frc.util.Pose;
import frc.util.terrorMath;

public class paths {

    public interface PATH_SET{
		Pose START = null;
		Waypoint[] WAYPOINTS = null;
    }
    
    public static class FRANTIC implements PATH_SET{
        public final static Pose START = new Pose(terrorMath.toMeters(7.5),terrorMath.toMeters(0),180);
        public static final Waypoint[] WAYPOINTS = {
          new Waypoint(-0.226678, 0.371058, 90, -0.3, false)};
    }
}
