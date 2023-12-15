package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveModule FL = new SwerveModule(
        DriveConstants.kFLDriveMotorPort,
        DriveConstants.kFLTurningMotorPort,
        DriveConstants.kFLDriveEncoderReversed,
        DriveConstants.kFLTurningEncoderReversed,
        DriveConstants.kFLDriveAbsoluteEncoderPort,
        DriveConstants.kFLDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFLDriveAbsoluteEncoderReversed
    );

    private final SwerveModule FR = new SwerveModule(
        DriveConstants.kFRDriveMotorPort,
        DriveConstants.kFRTurningMotorPort,
        DriveConstants.kFRDriveEncoderReversed,
        DriveConstants.kFRTurningEncoderReversed,
        DriveConstants.kFRDriveAbsoluteEncoderPort,
        DriveConstants.kFRDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFRDriveAbsoluteEncoderReversed
    );

    private final SwerveModule BL = new SwerveModule(
        DriveConstants.kBLDriveMotorPort,
        DriveConstants.kBLTurningMotorPort,
        DriveConstants.kBLDriveEncoderReversed,
        DriveConstants.kBLTurningEncoderReversed,
        DriveConstants.kBLDriveAbsoluteEncoderPort,
        DriveConstants.kBLDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBLDriveAbsoluteEncoderReversed
    );

    private final SwerveModule BR = new SwerveModule(
        DriveConstants.kBRDriveMotorPort,
        DriveConstants.kBRTurningMotorPort,
        DriveConstants.kBRDriveEncoderReversed,
        DriveConstants.kBRTurningEncoderReversed,
        DriveConstants.kBRDriveAbsoluteEncoderPort,
        DriveConstants.kBRDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBRDriveAbsoluteEncoderReversed
    );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());

    public SwerveSubsystem() {
        new Thread (() -> {
            while (true) {
                if (gyro.isConnected()) {
                    gyro.reset();
                    break;
                }
            }
            Shuffleboard.getTab("SmartDashboard").add(gyro);
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();

        System.out.println("RESETTED!");
    }

    public double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            FR.getPosition(),
            FL.getPosition(),
            BR.getPosition(),
            BL.getPosition()
      };
    }

    public void stopModules() {
        FL.stop();
        FR.stop();
        BL.stop();
        BR.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        FL.setDesiredState(desiredStates[0]);
        FR.setDesiredState(desiredStates[1]);
        BL.setDesiredState(desiredStates[2]);
        BR.setDesiredState(desiredStates[3]);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        Shuffleboard.getTab("SmartDashboard").add("LOCATION:", getPose().getTranslation().toString(), "0");
    }

}