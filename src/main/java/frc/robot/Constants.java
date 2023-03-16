// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.commands.TeleopSwerve;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    /* Gyro Constants */
    public static final int pigeonID = 5;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(22.00);
    public static final double wheelBase = Units.inchesToMeters(26.00);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) * (45.0/15.0); // 6.75:1 MK4i
    public static final double angleGearRatio = (150.0 / 7.0); // 

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, -trackWidth/ 2.0),
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth/ 2.0));

/*             What (all) others have...
          new Translation2d(wheelBase / 2.0, trackWidth/ 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth/ 2.0)); 
 * 
 *             What we have...
         new Translation2d(wheelBase / 2.0, -trackWidth/ 2.0),
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth/ 2.0));
 */




    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01; //.01
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second  //14.5 ft/s
    public static final double maxAngularVelocity = 6;            // 11.5

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;//false in 3512 code

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52.47); //50
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(278.34); //280
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(343.3); //347
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
    public static final int driveMotorID = 5;
    public static final int angleMotorID = 6;
    public static final int canCoderID = 4;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(347.6); //347
    public static final SwerveModuleConstants constants =
         new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public void setDefaultCommand(TeleopSwerve teleopSwerve) {
    }  
  }

//NOT swerve motor constants
  public final static int ELEVATOR_MOTOR_ID = 10;
  public final static int EXTENSION_MOTOR_ID = 11;
  public final static int INTAKE_MOTOR_ID = 12;
  public final static int ELEVATOR_FOLLOWER_MOTOR_ID = 13;

     
 //Pneumatics
  public final static int ELEVATOR_BRAKE_FORWARD = 0; 
  public final static int ELEVATOR_BRAKE_REVERSE = 1;
  // public final static int EXTENSION_BREAK_BACKWARD = 2;
  // public static final int EXTENSION_BRAKE_FORWARD = 3;
  public static final int EXTENSION_BRAKE_ON = 3;

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4; //3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; //3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }


}

