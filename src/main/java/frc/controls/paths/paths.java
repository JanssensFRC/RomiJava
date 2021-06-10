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
        public final static Pose START = new Pose(terrorMath.toMeters(0.0),terrorMath.toMeters(0.0), 0.0);
        public static final Waypoint[] WAYPOINTS = {
          //new Waypoint(-0.226678, 0.371058, 90.0, 0.5, false)};
          new Waypoint(-0.35, 0.25, -90.0, 0.5, false), // 1st Ball
          new Waypoint(0.4, 0.55, -97.0, -0.5, false),
          new Waypoint(0.35, 0.82, -263.0, -0.6, false)/*,
          new Waypoint(-0.08, 0.88, -272.0, -1.0, false), // 2nd Ball
          new Waypoint(0.46, 0.77, -297.0, 1.0, false),
          new Waypoint(0.59, 1.1, -406.0, 0.6, false),
          new Waypoint(0.2, 1.41, -414.0, 1.0, false), // 3rd Ball
        new Waypoint(0.56, 1.52, -496.0, -1.0, false)*/};
    }

    public static class ANTICS implements PATH_SET{
      public final static Pose START = new Pose(terrorMath.toMeters(0.0),terrorMath.toMeters(0.0), 0.0);
      public static final Waypoint[] WAYPOINTSONE = {
      new Waypoint(terrorMath.toMeters(0.0), terrorMath.toMeters(12.0), 0.0, 0.7, false),
      new Waypoint(terrorMath.toMeters(1.0), terrorMath.toMeters(38.0), 5.0, 0.8, false),
      new Waypoint(terrorMath.toMeters(1.5), terrorMath.toMeters(54.0), 10.0, 0.9, false),
       new Waypoint(terrorMath.toMeters(2.0), terrorMath.toMeters(70.0), 20.0, 1.0, true),
       new Waypoint(terrorMath.toMeters(-7.0), terrorMath.toMeters(60.0), 90.0, -0.3, false),
       new Waypoint(terrorMath.toMeters(-16.5), terrorMath.toMeters(76.5), 145.0, -0.3, true),
        // new Waypoint(terrorMath.toMeters(-15.0), terrorMath.toMeters(80), -45.0, 0.8, false)
      };

    }
}
