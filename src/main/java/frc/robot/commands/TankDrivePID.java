package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrivePID extends CommandBase{
    private final Drivetrain m_DT;

    private PIDController lPID = new PIDController(.55, 0.03, 0);
    private PIDController rPID = new PIDController(.55, 0.03, 0);

    public TankDrivePID(double leftSetpoint, double rightSetpoint, Drivetrain dt){
        lPID.setSetpoint(leftSetpoint);
        rPID.setSetpoint(rightSetpoint);
        m_DT = dt;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        m_DT.arcadeDrive(0.0, 0.0);
        m_DT.resetEncoders();
        lPID.setTolerance(0.125);
        rPID.setTolerance(0.125);
    }

    public void execute(){
        double lPIDOut = lPID.calculate(m_DT.getLeftDistanceInch(),lPID.getSetpoint());
        double rPIDOut = rPID.calculate(m_DT.getRightDistanceInch(),rPID.getSetpoint());

        m_DT.tankDrive(lPIDOut, rPIDOut);
    }

    public boolean isFinished(){
        return lPID.atSetpoint() && rPID.atSetpoint();
    }

    public void end(){
        m_DT.stop();
    }
}
