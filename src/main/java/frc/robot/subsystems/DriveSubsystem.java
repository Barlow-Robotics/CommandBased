// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PhysicsSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonSRX leftBackSide;
  WPI_TalonSRX leftFrontSide;
  WPI_TalonSRX rightFrontSide;
  WPI_TalonSRX rightBackSide;

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(
         leftFrontSide = new WPI_TalonSRX(DriveConstants.ID_leftFrontMotor),
         leftBackSide = new WPI_TalonSRX(DriveConstants.ID_leftBackMotor));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(
          rightFrontSide = new WPI_TalonSRX(DriveConstants.ID_rightFrontMotor),
          rightBackSide = new WPI_TalonSRX(DriveConstants.ID_rightBackMotor));

  // The robot's drive
 // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftFrontSide, rightFrontSide);

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry; //couldn't find differentialdrive/odometry in og code

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    leftBackSide.setInverted(false);
    leftFrontSide.setInverted(false);
    leftFrontSide.follow(leftBackSide);
    initializePIDConfig(leftBackSide);

    rightBackSide.setInverted(true);
    rightFrontSide.setInverted(true);
    rightFrontSide.follow(rightBackSide);
    initializePIDConfig(rightBackSide);
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    if (RobotBase.isSimulation()){
      simulationInit();
    }
  }
//stop here 02/19/21

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), leftBackSide.getSelectedSensorPosition(), rightBackSide.getSelectedSensorPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftBackSide.getSelectedSensorVelocity(), rightBackSide.getSelectedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  //    if (Math.abs(fwd) > 0.2 || Math.abs(rot) > 0.2) {
  //    System.out.printf("fwd= %6.4f, rot= %6.4f %n", fwd, rot);
  //  }
}

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftBackSide.setSelectedSensorPosition(0, 0, 0);
    rightBackSide.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftBackSide.getSelectedSensorPosition() + rightBackSide.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
 // public Encoder getLeftEncoder() {
 //   return m_leftEncoder;
  //}

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  private void initializePIDConfig(WPI_TalonSRX talon){
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveConstants.mainFeedbackLoop, Constants.DriveConstants.timeoutTime); //Encoder as feedback device, main PID loop, 30 ms timeout time
    talon.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
    talon.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
    talon.configNominalOutputForward(0);
    talon.configNominalOutputReverse(0);
    talon.configPeakOutputForward(1.0);
    talon.configPeakOutputReverse(-1.0);
    talon.configMotionCruiseVelocity((int)(Constants.DriveConstants.unitsPerRotation * Constants.DriveConstants.desiredRPMsForDrive));
    talon.config_kF(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainKf);
    talon.config_kP(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainkP);
    talon.config_kI(Constants.DriveConstants.PID_id, 0);
    talon.config_kD(Constants.DriveConstants.PID_id, 0);
  }

  private void simulationInit() {
      PhysicsSim.getInstance().addTalonSRX(rightFrontSide, 0.75, 4000, true);
      PhysicsSim.getInstance().addTalonSRX(leftFrontSide, 0.75, 4000, true);
      PhysicsSim.getInstance().addTalonSRX(rightBackSide, 0.75, 4000);
      PhysicsSim.getInstance().addTalonSRX(leftBackSide, 0.75, 4000);
  }

  public double getDistance(){
    double leftPos = leftFrontSide.getSelectedSensorPosition();
    double rightPos = rightFrontSide.getSelectedSensorPosition();
    double averagePos = (leftPos + rightPos)/2;
    return averagePos;
  }

  public void resetDistance(){
    leftFrontSide.setSelectedSensorPosition(0.0);
    rightFrontSide.setSelectedSensorPosition(0.0);
  }

}