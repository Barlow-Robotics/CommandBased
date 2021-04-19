// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int ID_leftFrontMotor = 2;
    public static final int ID_leftBackMotor = 1;
    public static final int ID_rightFrontMotor = 3;
    public static final int ID_rightBackMotor = 4;

    public static final double countsPerRevolution = 8192.0;
    public static final double circumferenceOfWheel = 8.0*Math.PI;
    public static final double distanceGoal = 120.0;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
    
    //Drivetrain
    public static final double drivetrainMinPower = 0.05;
    public static final double drivetrainMaxPower = 1.0;
    public static final double manualVoltageRampingConstant = 1.0;
    public static final double closedVoltageRampingConstant = 1.0;

    public static final int PID_id = 0;
    public static final double PID_Period = 1.0/20.0;
    public static final double DrivetrainKf = 0.1797; //0.1797
    public static final double DrivetrainkP = 0.02;

    public static final double unitsPerRotation = 8192;
    public static final double RPMsToUnitsPerHundredMilliseconds = 1.0/600.0;
    public static final double desiredRPMsForDrive = 500.0;
    public static final double maxDriveVelocity = 6000.0;
    public static final double VelocityInputConversionFactor = desiredRPMsForDrive * unitsPerRotation * RPMsToUnitsPerHundredMilliseconds;


    public static final int timeoutTime = 30;
    public static final int mainFeedbackLoop = 0;

    public static final double autoBackingDistance = 3.5; //3.5 rotations of the wheel ~ 65"
    public static final double pathFollowingThreshold = 20;
    public static final int autonomousDriveTime = 2500;
    public static final double autonomousDriveSpeed = 0.7;
    public static final double autonomousTurnRate = 0.7;

    public static final double speedConstantForBallChase = 0.3;
    public static final double maxAngleChangeForAlignFinish = 0.5;
    public static final double maxAngleDifferenceBetweenNavXAndVision = 0.01;
    public static final double alignTimeoutTime = 1000;
    public static final double alignMemorySize = 3;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int halfSpeedButton = 2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
