// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.util.Utils;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinCommand extends CommandBase {
  private final double m_desiredHeading;
  private final double m_speed;
  private final Drivetrain m_drive;
  private long m_startTime;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speed The speed which the robot will drive. Negative Value reverses the direction of the turn.
   * @param desiredHeading The desired heading to spin to.
   * @param drive The drivetrain subsystem on which this command will run
   */
  public SpinCommand(double speed, double desiredHeading, Drivetrain drive) {
    m_speed = speed; 
    m_desiredHeading = desiredHeading;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setOpenLoop(m_speed, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Utils.withinThreshold(m_drive.getGyroAngleZ()%360, m_desiredHeading, 5);
  }
}
