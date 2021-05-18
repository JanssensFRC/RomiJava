package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Spots2And3 extends SequentialCommandGroup {

    public Spots2And3(Drivetrain drivetrain){
        addCommands(new TankDriveTime(1.0, 1.0, 1.45, drivetrain),
                new TankDriveTime(0.0, 0.0, 0.5, drivetrain),
                new TankDriveTime(-0.75, -0.75, 1.0, drivetrain),
                new TankDriveTime(0.0, 0.0, 0.25, drivetrain),
                new TankDriveTime(0.8, 1.0, 0.75, drivetrain),
                new TankDriveTime(0.9, 0.8, 0.75, drivetrain)
                );
    }
    
}
