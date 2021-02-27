// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.controls.paths.paths;
import frc.controls.paths.paths.MADNESS;
import frc.robot.subsystems.Drivetrain;
import frc.util.terrorMath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TerrorAuto extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public TerrorAuto(Drivetrain drivetrain) {
    addCommands(new TankDriveTime(-1.0, -1.0, 0.24, drivetrain),
                new TankDriveTime(-1.0, -0.64, 1.07, drivetrain),
                new TankDriveTime(-1.0, -0.45, 0.50, drivetrain),
                new TankDriveTime(-1.0, -1.0, 0.65, drivetrain),
                new TankDriveTime(-0.3, -1.0, 0.42, drivetrain),
                new TankDriveTime(-0.7, -1.0, 1.2, drivetrain)//,
                /*new TankDriveTime(-1.0, -1.0, 0.29, drivetrain)*/);
  }
}
