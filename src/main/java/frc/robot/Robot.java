/*
gyro
derivitaves of position
*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  public static final double DISTANCE_PER_ROTATION = 1.0d/8.45d * 6.0d * Math.PI; // inches

  CANSparkMax front_left;
	CANSparkMax rear_left;
	CANSparkMax front_right;
	CANSparkMax rear_right;

	SpeedControllerGroup left_a;
  SpeedControllerGroup right_a;
  
  DifferentialDrive driveTrain;

  CANEncoder encoderFrontLeft, encoderRearLeft, encoderFrontRight, encoderRearRight;

  ADXRS450_Gyro gyro;

  Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    front_left = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
		rear_left = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
		front_right = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    rear_right = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    left_a = new SpeedControllerGroup(front_left, rear_left);
    right_a = new SpeedControllerGroup(front_right, rear_right);

    driveTrain = new DifferentialDrive(left_a, right_a);

    encoderFrontLeft = front_left.getEncoder();
    encoderFrontRight = front_right.getEncoder();

    encoderRearLeft = rear_left.getEncoder();
    encoderRearRight = rear_right.getEncoder();

    gyro = new ADXRS450_Gyro();

    SmartDashboard.putNumber("KP", 0.01);
    SmartDashboard.putNumber("set point", 100);

  }



  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Distance", getAverageDistance());
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();

    gyro.reset();

    encoderFrontLeft.setPosition(0.0d);
		encoderRearLeft.setPosition(0.0d);
		encoderFrontRight.setPosition(0.0d);
		encoderRearRight.setPosition(0.0d);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    SmartDashboard.putNumber("Distance", getAverageDistance());
    SmartDashboard.putNumber("Speed (front left)", encoderFrontLeft.getVelocity());

    safeArcadeDrive( (getAverageDistance() - SmartDashboard.getNumber("set point", 100)) * SmartDashboard.getNumber("KP", 0) , 0.0d);

    // if (getAverageDistance() < 1) {
    //   driveTrain.arcadeDrive(0.4, 0);
    // }
    // else {
    //   driveTrain.arcadeDrive(0, 0);
    // }

    // gyro.getAngle();

  }

  private double getLeftDistance() {
    double leftDistance = encoderFrontLeft.getPosition() + encoderRearLeft.getPosition();
    leftDistance /= 2;
    leftDistance *= DISTANCE_PER_ROTATION;

    return -leftDistance;
  }

  private double getRightDistance() {
    double rightDistance = encoderFrontRight.getPosition() + encoderRearRight.getPosition();
    rightDistance /= 2;
    rightDistance *= DISTANCE_PER_ROTATION;

    return rightDistance;
  }

  private double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  private void safeArcadeDrive(double speed, double turn) {
    double outSpeed = 0;
    double outTurn = 0;

    if (Math.abs(speed) < 0.1) {
      outSpeed = 0.0;
    }
    else {
      if (Math.abs(speed) > 0.7) {
        outSpeed = Math.signum(speed) * 0.7;
      }
      if (Math.abs(speed) < 0.3) {
        outSpeed = Math.signum(speed) * 0.3;
      }
    }
    driveTrain.arcadeDrive(outSpeed, outTurn);
    SmartDashboard.putNumber("out speed", outSpeed);
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
