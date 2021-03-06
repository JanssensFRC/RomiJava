// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.controls.paths.paths;
import frc.controls.paths.paths.ANTICS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Spots2And3;
import frc.util.terrorMath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TerrorAuto extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public TerrorAuto(Drivetrain drivetrain) {
  //   addCommands();
  // }
        super(
          new InstantCommand(() -> drivetrain.resetEncoders()),
        new DrivePath(drivetrain, false, ANTICS.START, ANTICS.WAYPOINTSONE),
        new SpinCommand(0.3, 180.0, drivetrain),
        new PrintCommand("Finished Autonomous Routine at "+Timer.getFPGATimestamp())
        /*new RunCommand(()->drivetrain.setOpenLoop(0.8, -0.4), drivetrain).withTimeout(1.75)*/);
    }

}


