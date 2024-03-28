// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  public static class ModuleConstants {
    // L3 swerve bizimki

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  }

  public static class DriveConstants {

    public static final double kSagSolArasi = 0.51435;
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.259428, kSagSolArasi / 2),
        new Translation2d(0.259428, -kSagSolArasi / 2),
        new Translation2d(-0.102523, kSagSolArasi / 2),
        new Translation2d(-0.102523, -kSagSolArasi / 2)
    ); 

    public static final int kFLDriveMotorPort = 5;
    public static final int kBLDriveMotorPort = 9;
    public static final int kFRDriveMotorPort = 14;
    public static final int kBRDriveMotorPort = 13;

    public static final int kFLTurningMotorPort = 4;
    public static final int kBLTurningMotorPort = 8;
    public static final int kFRTurningMotorPort = 62;
    public static final int kBRTurningMotorPort = 2;

    public static final boolean kFLTurningEncoderReversed = true;
    public static final boolean kBLTurningEncoderReversed = true;
    public static final boolean kFRTurningEncoderReversed = true;
    public static final boolean kBRTurningEncoderReversed = true;

    public static final boolean kFLDriveEncoderReversed = true;
    public static final boolean kBLDriveEncoderReversed = true;
    public static final boolean kFRDriveEncoderReversed = false;
    public static final boolean kBRDriveEncoderReversed = false;

    public static final int kFLTurningAbsoluteEncoderPort = 51; 
    public static final int kBLTurningAbsoluteEncoderPort = 50; 
    public static final int kFRTurningAbsoluteEncoderPort = 52;
    public static final int kBRTurningAbsoluteEncoderPort = 53; 

    public static final double kFLTurningAbsoluteEncoderOffset = -0.422363;
    public static final double kBLTurningAbsoluteEncoderOffset = -0.129883;
    public static final double kFRTurningAbsoluteEncoderOffset = -0.845703;
    public static final double kBRTurningAbsoluteEncoderOffset = -0.775879;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.65;
    public static final double kTeleDriveBoostSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.9;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2 / 1.66;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveBoostMaxAccelerationUnitsPerSecond = 1.5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kTeleDriveSlowModeMultiplier = (1.0/8.0);

  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kPXYController = 3.2;
    public static final double kPThetaController = 2;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, 3, Math.toRadians(540), Math.toRadians(720));
  }

  public static class OIConstants {
    public static final double kDeadband = 0.12;
  }

  public static class VisionConstants {
    public static final double kLimeLightMountAngleRadians = Math.toRadians(-30);
    public static final double kLimeLightHeightMeters = 0.26;

    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.294428, 0.184428, kLimeLightHeightMeters), new Rotation3d(0, kLimeLightMountAngleRadians, 0));
    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final Matrix<N3, N1> kTagStdDevs = VecBuilder.fill(4, 4, 1);
  }

  public static class PIDConstants{
    public static final double kPTurning = 0.45;
    public static final double kDTurning = 0.001;

    public static final double kPLimeLightRotate = 0.034;
    public static final double kDLimeLightRotate = 0;

    public static final double kP180Rotate = 0.045;
    public static final double kD180Rotate = 0.00001;
  }

  public static class ShooterConstants{
    
    public static final double kGearRatio = 1.0 / 11.357; // CANCODERIN GEAR RATIO
    public static final double kPivotMotorRot2Rad = kGearRatio * 2 * Math.PI;
    public static final float kMinShooterAngle = 0;
    public static final float kMaxShooterAngle = 39;

    public static final int kPivotMotorId = 11;
    public static final int kAbsoluteEncoderId = 54;

    public static final double kAbsoluteEncoderOffset = -0.028320;
    public static final boolean kPivotMotorReversed = true;

    public static final double kAngleP = 1;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleToleranceRad = Math.toRadians(0.05);

    public static final double k0mAngle = 26; //26
    public static final double k05mAngle = 20.1; //20.1
    public static final double k1mAngle = 14; //14
    public static final double k15mAngle = 7; //7
    public static final double k2mAngle = 4.4; //4.4 
    public static final double k25mAngle = 0.9; //0.9

    ////////////////////////////////////////////////////////////////////////////////
    
    
    public static final int kShooterMotorLeftId = 1;
    public static final int kShooterMotorRightId = 3;    
  
    public static final boolean kShooterMotorLeftReversed = false;
    public static final boolean kShooterMotorRightReversed = true; 
  
    public static final double kSpeakerSpeedLeft = 0.9;
    public static final double kSpeakerSpeedRight = 0.6;

    public static final double kAmpSpeedLeft = 0.50;
    public static final double kAmpSpeedRight = 0.50;
  }

  public static class IntakextenderConstants{
    public static final int kIntakeMotorId = 10;
    public static final boolean kIntakeMotorReversed = false;
    public static final double kIntakeMotorSpeed = 0.5;
    public static final double kIntakeDeadband = 0.3;


    public static final int kExtenderMotorId = 6;
    public static final boolean kExtenderMotorReversed = false;
  }
} 
