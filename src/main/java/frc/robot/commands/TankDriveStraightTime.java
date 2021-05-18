// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveStraightTime extends CommandBase {
  private final double m_duration;
  private final double m_Lspeed;
  private final double m_Rspeed;
  private final Drivetrain m_drive;
  private long m_startTime;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param time How much time to drive in seconds
   * @param drive The drivetrain subsystem on which this command will run
   */
  public TankDriveStraightTime(double lSpeed, double rSpeed, double time, Drivetrain drive) {
    m_Lspeed = lSpeed;
    m_Rspeed = rSpeed;
    m_duration = time * 1000;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double LOutput = m_Lspeed * (m_drive.getRightDistanceInch()+1/m_drive.getLeftDistanceInch()+1);
    double ROutput = m_Rspeed * (m_drive.getLeftDistanceInch()+1/m_drive.getRightDistanceInch()+1);
    m_drive.tankDrive(LOutput, ROutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
