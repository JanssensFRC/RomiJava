package frc.robot.commands;


import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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

    public void updatePosTrack(Drivetrain dt){
        
    }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        SmartDashboard.putNumber("X Position (Vince)", xPosition);
        SmartDashboard.putNumber("Y Position (Vince)", yPosition);
    }
}
