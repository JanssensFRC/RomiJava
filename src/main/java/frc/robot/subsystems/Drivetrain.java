// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controls.CubicSplineFollower;
import frc.robot.sensors.RomiGyro;
import frc.util.Tuple;
import frc.util.Utils;
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
  
  public double theta;
    public double xPosition;
    public double yPosition;
    public double L;
    public double R;
    private double dTheta;
    private double dL;
    private double dR;
    private double intTheta;
    private double intL;
    private double intR;
    private double r;
    private double arcDist;
    private double linDist;
    private double absAng;
    private final double wheelDist = 2.75;
    private double m_quickStopThreshold = 0.2;
    private double m_quickStopAlpha = 0.1;
    private double m_quickStopAccumulator;

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
    L = getLeftDistanceInch();
    R = getRightDistanceInch();
    SmartDashboard.putNumber("Left Encoder", L);
    SmartDashboard.putNumber("Right Encoder", R);
    dL = L - intL;  //get distance traveled since last execute, to find effective arc length assuming constant velocity
    dR = R - intR;
    SmartDashboard.putNumber("dL", dL);
    SmartDashboard.putNumber("dR", dR);
    arcDist = (dL + dR) / 2; //get arc length of midpoint arc of both wheels, will be the average
    if (dR == dL) { //going precisely straight, extreme edge case
        linDist = arcDist;
        absAng = intTheta;
    } 
    else {
        r = wheelDist * (Math.abs((dL + dR) / (dL - dR)));// + Math.abs((dL + dR) / (dR - dL))) / 2; //solving for midpoint arc radius using system of arc length equations
        dTheta = Math.copySign((arcDist / r), (dR - dL)); //calculate angle turned using arc length formula after solving for radius
        theta += dTheta; //increment absolute angle by turn angle
        linDist = 2 * r * Math.cos(Math.abs(Math.PI / 2 + (dTheta / 2))); // using isoceles triangle base formula, calculate
                                                                    // the linear distance between start and endpoint of
                                                                    // the midpoint arc
        absAng = intTheta + dTheta / 2 + Math.PI / 2; //absolute angle between start and endpoint of midpoint arc
    }
    xPosition += linDist * Math.sin(absAng);
    yPosition += linDist * Math.cos(absAng);

    intL = L; //store last encoder positions/distances and absoulute angle
    intR = R;
    //resetEncoders();
    intTheta = theta;

		double leftSpeed = m_leftEncoder.getDistance()/time;
		double rightSpeed = m_rightEncoder.getDistance()/time;
		model.updateSpeed(leftSpeed, rightSpeed, time);
		model.updateHeading(theta);
    model.updatePosition(time);

    SmartDashboard.putNumber("Theta", Math.toDegrees(theta));
    SmartDashboard.putNumber("X Pos", xPosition);
    SmartDashboard.putNumber("Y Pos", yPosition);
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

  public void terribleDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		double m_deadband = 0.02;
		xSpeed = Utils.limit(xSpeed);
		xSpeed = Utils.applyDeadband(xSpeed, m_deadband);

		zRotation = Utils.limit(zRotation);
		zRotation = Utils.applyDeadband(zRotation, m_deadband);

		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(xSpeed) < m_quickStopThreshold) {
              m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
						+ m_quickStopAlpha * zRotation * 2;
			}
			overPower = true;
			angularPower = zRotation;
		} else {
			overPower = false;
			angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1) {
				m_quickStopAccumulator -= 1;
			} else if (m_quickStopAccumulator < -1) {
				m_quickStopAccumulator += 1;
			} else {
				m_quickStopAccumulator = 0.0;
			}
		}

		double leftOutput = xSpeed + angularPower;
		double rightOutput = xSpeed - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftOutput > 1.0) {
				rightOutput -= leftOutput - 1.0;
				leftOutput = 1.0;
			} else if (rightOutput > 1.0) {
				leftOutput -= rightOutput - 1.0;
				rightOutput = 1.0;
			} else if (leftOutput < -1.0) {
				rightOutput -= leftOutput + 1.0;
				leftOutput = -1.0;
			} else if (rightOutput < -1.0) {
				leftOutput -= rightOutput + 1.0;
				rightOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
		if (maxMagnitude > 1.0) {
			leftOutput /= maxMagnitude;
			rightOutput /= maxMagnitude;
		}

		setOpenLoop(leftOutput, rightOutput);
  }
  
  private void setOpenLoop(double left, double right) {
		if (controlState != DriveControlState.OPEN_LOOP) {
			System.out.println("Switching to open loop control, time: " + time);
			controlState = DriveControlState.OPEN_LOOP;
		}
		m_leftMotor.set(right);
		m_rightMotor.set(left);
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
