package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private PWM frontServo, backServo;
    
    public Intake(){
        frontServo = new PWM(4);
        backServo = new PWM(3);
    }

    public void runServoFetch(){
        frontServo.setSpeed(1.0);
        backServo.setSpeed(-1.0);
    }
}
