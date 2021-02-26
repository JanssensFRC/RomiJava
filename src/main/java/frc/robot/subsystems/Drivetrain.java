// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controls.CubicSplineFollower;
import frc.robot.sensors.RomiGyro;
import frc.util.Tuple;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  protected static final int UPDATE_RATE = 200;
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  public CubicSplineFollower waypointNav;
  public DrivetrainModel model;

  private double time;

	private DriveControlState controlState = DriveControlState.DISABLED;

	private enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		PATH_FOLLOWING, // velocity PID control
		DISABLED,
	}

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    model = new DrivetrainModel();
		model.setPosition(0.0, 0.0, 0.0);

    waypointNav = new CubicSplineFollower(model);
    this.updateThreadStart();
  }

  protected void update() {
		double time = Timer.getFPGATimestamp();
		double deltaTime = time - this.time;
		this.time = time;
		this.updateOdometry(deltaTime);

		switch (controlState) {
			case OPEN_LOOP:
				break;
			case PATH_FOLLOWING:
				driveWaypointNavigator();
				break;
			case DISABLED:
				break;
		}
  }
  
  private void updateOdometry(double time) {
		double leftSpeed = m_leftEncoder.getDistance()/time;
		double rightSpeed = m_rightEncoder.getDistance()/time;
		model.updateSpeed(leftSpeed, rightSpeed, time);
		model.updateHeading(m_gyro.getAngleX());
		model.updatePosition(time);
  }
  
  public void startPathFollowing() {
		if (controlState != DriveControlState.PATH_FOLLOWING) {
			System.out.println("Switching to path following, time: " + time);
			controlState = DriveControlState.PATH_FOLLOWING;
		}
		// BANANA TODO print waypoints
	}

  private void driveWaypointNavigator() {
		Tuple output = waypointNav.updatePursuit(model.center);
		double leftSpeed = output.left;
    double rightSpeed = output.right;
    
    setSpeed(leftSpeed, rightSpeed);
  }
  
  private void setSpeed(double left, double right) {
		m_leftMotor.set(left);
		m_rightMotor.set(right);
	}

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public void stop(){
    m_diffDrive.tankDrive(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  protected void updateThreadStart() {
    Thread t = new Thread(() -> {
        while (!Thread.interrupted()) {
            this.update();
            try {
                Thread.sleep(1000 / UPDATE_RATE);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    });
    t.start();
  }
}
