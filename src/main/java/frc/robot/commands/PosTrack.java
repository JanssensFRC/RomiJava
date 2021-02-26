package frc.robot.commands;


import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;

public class PosTrack extends CommandBase {
    public double theta;
    public double xPosition;
    public double yPosition;
    public double L;
    public double R;
    private double dTheta;
    private double dL;
    private double dR;
    private double intTheta;
    private double intL;
    private double intR;
    private double r;
    private double arcDist;
    private double linDist;
    private double absAng;
    private final double wheelDist = 2.75;
    private final Drivetrain m_drive;
    
    public PosTrack(Drivetrain drive) {
        m_drive = drive;
        
    }


    public void initialize() {
        theta = 0;
        xPosition = 0;
        yPosition = 0;
        L = 0;
        R = 0;
        intTheta = 0;
        intL = 0;
        intR = 0;
        dTheta = 0;
        dL = 0;
        dR = 0;
        r = 0;
        arcDist = 0;
        linDist = 0;
        absAng = 0;
        m_drive.resetEncoders();
    }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        L = m_drive.getLeftDistanceInch();
        R = m_drive.getRightDistanceInch();
        dL = L - intL;  //get distance traveled since last execute, to find effective arc length assuming constant velocity
        dR = R - intR;
        arcDist = (dL + dR) / 2; //get arc length of midpoint arc of both wheels, will be the average
        if (dR == dL) { //going precisely straight, extreme edge case
            linDist = arcDist;
            absAng = intTheta;
        } else {
            r = wheelDist * (Math.abs((dL + dR) / (dL - dR)) + Math.abs((dL + dR) / (dR - dL))) / 2; //solving for midpoint arc radius using system of arc length equations
            dTheta = Math.copySign((arcDist / r), (dR - dL)); //calculate angle turned using arc length formula after solving for radius
            theta += dTheta; //increment absolute angle by turn angle
            linDist = 2 * r * Math.cos(Math.abs(dTheta)); //using isoceles triangle base formula, calculate the linear distance between start and endpoint of the midpoint arc
            absAng = intTheta + (dTheta / 2); //absolute angle between start and endpoint of midpoint arc
        }
        xPosition += r * Math.sin(absAng);
        yPosition += r * Math.cos(absAng);

        intL = L; //store last encoder positions/distances and absoulute angle
        intR = R;
        intTheta = theta;
    }
}
