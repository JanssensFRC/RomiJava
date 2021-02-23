// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TerrorAuto extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public TerrorAuto(Drivetrain drivetrain) {
    addCommands(
        new DriveArcade(-0.8, -0.3, 12.5, drivetrain),
        new DriveArcade(-0.5, -0.5, 4, drivetrain),
        new DriveArcade(-0.8, -0.3, 8, drivetrain),
        new DriveArcade(-0.5, -0.7, 3, drivetrain),
        new DriveArcade(-0.8, -0.5, 2.5, drivetrain),
        new DriveArcade(-0.8, -0.4, 12, drivetrain),
        new DriveArcade(-0.8, 0.5, 9, drivetrain)
    );
  }
}
