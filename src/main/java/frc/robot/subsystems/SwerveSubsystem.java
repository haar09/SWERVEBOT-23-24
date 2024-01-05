package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

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
    private final Field2d field = new Field2d();

    public SwerveSubsystem() {
        new Thread (() -> {
            while (true) {
                if (gyro.isConnected()) {
                    gyro.reset();
                    break;
                }
            }
        }).start();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Gyro", gyro);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
              builder.setSmartDashboardType("SwerveDrive");
          
              builder.addDoubleProperty("Front Left Angle", () -> FL.getTurningPosition() / ModuleConstants.kTurningEncoderRot2Rad, null);
              builder.addDoubleProperty("Front Left Velocity", () -> FL.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Front Right Angle", () -> -FR.getTurningPosition() / ModuleConstants.kTurningEncoderRot2Rad, null);
              builder.addDoubleProperty("Front Right Velocity", () -> FR.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Back Left Angle", () -> -BL.getTurningPosition() / ModuleConstants.kTurningEncoderRot2Rad, null);
              builder.addDoubleProperty("Back Left Velocity", () -> BL.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Back Right Angle", () -> BR.getTurningPosition() / ModuleConstants.kTurningEncoderRot2Rad, null);
              builder.addDoubleProperty("Back Right Velocity", () -> BR.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Robot Angle", () -> getHeading(), null);
            }
          });
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
        field.setRobotPose(new Pose2d(-getPose().getY(), getPose().getX(), getPose().getRotation()));
    }

    public void switchIdleMode(){
        FL.switchIdleMode();
        FR.switchIdleMode();
        BL.switchIdleMode();
        BR.switchIdleMode();
    }

}