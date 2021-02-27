// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_SpeedSupplier;
  private final Supplier<Double> m_turnSpeedSupplier;
  double turnCoef;
  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public TankDrive(
      Drivetrain drivetrain,
      Supplier<Double> SpeedSupplier,
      Supplier<Double> turnSpeedSupplier) {
    m_drivetrain = drivetrain;
    m_SpeedSupplier = SpeedSupplier;
    m_turnSpeedSupplier = turnSpeedSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_SpeedSupplier.get() > 0.7){
      turnCoef = 0.27;
    } else {
      turnCoef = 0.55;
    }
    // turnCoef = (1.5 - (m_SpeedSupplier.get() * m_SpeedSupplier.get())) * 0.7;
    // if (m_SpeedSupplier.get() < 0.3){
    //   turnCoef = 0.65;
    // }
    // if (m_turnSpeedSupplier.get() > 0) {
    m_drivetrain.tankDrive(Math.min(Math.max(m_SpeedSupplier.get() - m_turnSpeedSupplier.get() * turnCoef, -1), 1), Math.min(Math.max(m_SpeedSupplier.get() + m_turnSpeedSupplier.get() * turnCoef, -1), 1));
    // } else {
    //   m_drivetrain.tankDrive(m_SpeedSupplier.get(), m_SpeedSupplier.get() + m_turnSpeedSupplier.get() * turnCoef);
    // }
    //m_drivetrain.tankDrive(m_SpeedSupplier.get(), m_turnSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
