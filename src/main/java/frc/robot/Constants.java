// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public static final double kSagSolArasi = 56.515;
    public static final double kOnArkaArasi = 56.515;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kOnArkaArasi / 2, kSagSolArasi / 2),
        new Translation2d(kOnArkaArasi / 2, -kSagSolArasi / 2),
        new Translation2d(-kOnArkaArasi / 2, kSagSolArasi / 2),
        new Translation2d(-kOnArkaArasi / 2, -kSagSolArasi / 2)
    ); 

    public static final int kFLDriveMotorPort = 1;
    public static final int kBLDriveMotorPort = 12;
    public static final int kFRDriveMotorPort = 62;
    public static final int kBRDriveMotorPort = 14;

    public static final int kFLTurningMotorPort = 3;
    public static final int kBLTurningMotorPort = 13;
    public static final int kFRTurningMotorPort = 2;
    public static final int kBRTurningMotorPort = 15;

    public static final boolean kFLTurningEncoderReversed = true;
    public static final boolean kBLTurningEncoderReversed = true;
    public static final boolean kFRTurningEncoderReversed = true;
    public static final boolean kBRTurningEncoderReversed = true;

    public static final boolean kFLDriveEncoderReversed = false;
    public static final boolean kBLDriveEncoderReversed = false;
    public static final boolean kFRDriveEncoderReversed = true;
    public static final boolean kBRDriveEncoderReversed = true;

    public static final int kFLDriveAbsoluteEncoderPort = 50;
    public static final int kBLDriveAbsoluteEncoderPort = 53;
    public static final int kFRDriveAbsoluteEncoderPort = 51;
    public static final int kBRDriveAbsoluteEncoderPort = 52;

    public static final boolean kFLDriveAbsoluteEncoderReversed = true; //unused
    public static final boolean kBLDriveAbsoluteEncoderReversed = true; //unused
    public static final boolean kFRDriveAbsoluteEncoderReversed = true; //unused
    public static final boolean kBRDriveAbsoluteEncoderReversed = true; //unused

    public static final double kFLDriveAbsoluteEncoderOffsetDeg = -0.422;
    public static final double kBLDriveAbsoluteEncoderOffsetDeg = -0.128;
    public static final double kFRDriveAbsoluteEncoderOffsetDeg = 0.286;
    public static final double kBRDriveAbsoluteEncoderOffsetDeg = -0.097;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 35;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 25;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kTeleDriveSlowModeMultiplier = (1.0/8.0);

  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 9;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    /* AYARLA BUNLARI OLM */
    public static final double kPXController = 4;
    public static final double kPYController = 5;
    public static final double kPThetaController = 0.75;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static class OIConstants {
    public static final double kDeadband = 0.12;
    public static final double kLimeLightMountAngleDegrees = 20;
    public static final double kLimeLightHeightMeters = 0.175;
    public static final double kGoalHeightMeters = 0.54;
  }

  public static class PIDConstants{
    public static final double kPTurning = 0.75;
    public static final double kPLimeLightRotate = 0.005;
  }

} 
