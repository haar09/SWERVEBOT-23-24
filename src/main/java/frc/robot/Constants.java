// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

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
    public static final double kOnArkaArasi = 0.36195;
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
    public static final boolean kBLDriveEncoderReversed = true;
    public static final boolean kFRDriveEncoderReversed = false;
    public static final boolean kBRDriveEncoderReversed = false;

    public static final int kFLTurningAbsoluteEncoderPort = 51; 
    public static final int kBLTurningAbsoluteEncoderPort = 50; 
    public static final int kFRTurningAbsoluteEncoderPort = 52;
    public static final int kBRTurningAbsoluteEncoderPort = 53; 

    public static final double kFLTurningAbsoluteEncoderOffset = 0.037842;
    public static final double kBLTurningAbsoluteEncoderOffset = -0.179688;
    public static final double kFRTurningAbsoluteEncoderOffset = 0.152100;
    public static final double kBRTurningAbsoluteEncoderOffset = -0.377686;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2 ;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kTeleDriveSlowModeMultiplier = (1.0/8.0);

  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kPXYController = 3;
    public static final double kPThetaController = 2;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, 3.0, Math.toRadians(540), Math.toRadians(720));

    public static Pose2d kAmpPose;

    static {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            kAmpPose = new Pose2d(14.6, 7.5, Rotation2d.fromDegrees(90)); //DEĞİŞÇEK
        } else {
            kAmpPose = new Pose2d(1.9, 7.5, Rotation2d.fromDegrees(90)); //DEĞİŞÇEK
        }
    }
  }

  public static class OIConstants {
    public static final double kDeadband = 0.12;
  }

  public static class VisionConstants {
    public static final double kLimeLightMountAngleRadians = Math.toRadians(-20); //DEĞİŞÇEK
    public static final double kLimeLightHeightMeters = 0.17; //DEĞİŞÇEK

    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.34, 0.0, kLimeLightHeightMeters), new Rotation3d(0, kLimeLightMountAngleRadians, 0));
    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
    public static final double kTXTolerance = 2.5;
  }

  public static class PIDConstants{
    public static final double kPTurning = 0.45;
    public static final double kPLimeLightRotate = 0.05;
  }

  public static class ShooterConstants{
    /*
    public static final double kGearRatio = 1.0 / 96.0814286;
    public static final double kPivotMotorRot2Rad = kGearRatio * 2 * Math.PI;
    public static final float kMinShooterAngleRad = (float)Math.toRadians(0);
    public static final float kMaxShooterAngleRad = (float)Math.toRadians(30);

    public static final double kPivotToShooterMouthDegrees = ??;
    public static final double kPivotToShooterMouthMeters = ??;
    public static final double kPivotHeightMeters = ??;
    public static final double kPivotToCameraXDistanceMeters = ??;

    public static final int kPivotMotorId = ??; // ELEKTRİKLE AYARLANACAK
    public static final int kAbsoluteEncoderId = ??; // ELEKTRİKLE AYARLANACAK

    public static final double kAbsoluteEncoderOffset = ??; // TAKILINCA AYARLANACAK
    public static final boolean kPivotMotorReversed = false; // TAKILINCA AYARLANACAK

    public static final double kAngleP = 0;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleToleranceRad = Math.toRadians(2);
    public static final double kAngleH = 0;

    ////////////////////////////////////////////////////////////////////////////////

    public static final int kShooterMotorLeftId = ??; // ELEKTRİKLE AYARLANACAK
    public static final int kShooterMotorRightId = ??; // ELEKTRİKLE AYARLANACAK    
  
    public static final boolean kShooterMotorLeftReversed = false; // TAKILINCA AYARLANACAK
    public static final boolean kShooterMotorRightReversed = false; // TAKILINCA AYARLANACAK
    
    public static final double kShooterMotorRPMtoPercentage = 1.0 / 5676.0;

    public static final double kShooterMotorLeftSpeed = 0.9; //TAKILINCA AYARLANACAK
    public static final double kShooterMotorRightSpeed = 0.9; //TAKILINCA AYARLANACAK
    */
  }

  public static class IntakextenderConstants{
    public static final int kIntakeMotorId = 8; // ELEKTRİKLE AYARLANACAK
    public static final boolean kIntakeMotorReversed = false; // TAKILINCA AYARLANACAK

    public static final int kExtenderMotorId = 4; //ELEKTRİKLE AYARLANACAK
    public static final boolean kExtenderMotorReversed = false; //TAKILINCA AYARLANACAK
    
  }
} 
