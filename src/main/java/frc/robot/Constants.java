// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public static final int kFLDriveMotorPort = 62;
    public static final int kBLDriveMotorPort = 1;
    public static final int kFRDriveMotorPort = 14;
    public static final int kBRDriveMotorPort = 12;

    public static final int kFLTurningMotorPort = 2;
    public static final int kBLTurningMotorPort = 3;
    public static final int kFRTurningMotorPort = 15;
    public static final int kBRTurningMotorPort = 13;

    public static final boolean kFLTurningEncoderReversed = true;
    public static final boolean kBLTurningEncoderReversed = true;
    public static final boolean kFRTurningEncoderReversed = true;
    public static final boolean kBRTurningEncoderReversed = true;

    public static final boolean kFLDriveEncoderReversed = true;
    public static final boolean kBLDriveEncoderReversed = false;
    public static final boolean kFRDriveEncoderReversed = true;
    public static final boolean kBRDriveEncoderReversed = false;

    public static final int kFLTurningAbsoluteEncoderPort = 51; 
    public static final int kBLTurningAbsoluteEncoderPort = 50; 
    public static final int kFRTurningAbsoluteEncoderPort = 52; 
    public static final int kBRTurningAbsoluteEncoderPort = 53; 

    public static final double kFLTurningAbsoluteEncoderOffsetDeg = 0.049;
    public static final double kBLTurningAbsoluteEncoderOffsetDeg = 0.316;
    public static final double kFRTurningAbsoluteEncoderOffsetDeg = -0.347;
    public static final double kBRTurningAbsoluteEncoderOffsetDeg = -0.375;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4 / 25;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kTeleDriveSlowModeMultiplier = (1.0/8.0);

  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kPXYController = 3; //AYARLI
    public static final double kPThetaController = 0.0001; //AYARLI DEĞİL
  }

  public static class OIConstants {
    public static final double kDeadband = 0.12;
    public static final double kLimeLightMountAngleDegrees = 20;
    public static final double kLimeLightHeightMeters = 0.175;
    public static final double kGoalHeightMeters = 0.54;
  }

  public static class PIDConstants{
    public static final double kPTurning = 0.75;
    public static final double kPLimeLightRotate = 0.0005;
  }

} 
