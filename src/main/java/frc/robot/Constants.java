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
    public static final double kPTurning = 0.5;
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

    public static final boolean kFLDriveEncoderReversed = true;
    public static final boolean kBLDriveEncoderReversed = true;
    public static final boolean kFRDriveEncoderReversed = false;
    public static final boolean kBRDriveEncoderReversed = false;

    /* robot bitince ayarlanacak */
    public static final int kFLDriveAbsoluteEncoderPort = 0;
    public static final int kBLDriveAbsoluteEncoderPort = 2;
    public static final int kFRDriveAbsoluteEncoderPort = 1;
    public static final int kBRDriveAbsoluteEncoderPort = 3;

    public static final boolean kFLDriveAbsoluteEncoderReversed = false;
    public static final boolean kBLDriveAbsoluteEncoderReversed = false;
    public static final boolean kFRDriveAbsoluteEncoderReversed = false;
    public static final boolean kBRDriveAbsoluteEncoderReversed = false;

    /* robot bitince ayarlanacak */
    public static final double kFLDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBLDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFRDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBRDriveAbsoluteEncoderOffsetRad = 0;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

  }
  
  public static class OIConstants {
    public static final double kDeadband = 0.05;
  }

} 
