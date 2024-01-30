package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveModule FL = new SwerveModule(
        DriveConstants.kFLDriveMotorPort,
        DriveConstants.kFLTurningMotorPort,
        DriveConstants.kFLDriveEncoderReversed,
        DriveConstants.kFLTurningEncoderReversed,
        DriveConstants.kFLDriveAbsoluteEncoderPort,
        DriveConstants.kFLDriveAbsoluteEncoderOffsetDeg,
        DriveConstants.kFLDriveAbsoluteEncoderReversed
    );

    private final SwerveModule FR = new SwerveModule(
        DriveConstants.kFRDriveMotorPort,
        DriveConstants.kFRTurningMotorPort,
        DriveConstants.kFRDriveEncoderReversed,
        DriveConstants.kFRTurningEncoderReversed,
        DriveConstants.kFRDriveAbsoluteEncoderPort,
        DriveConstants.kFRDriveAbsoluteEncoderOffsetDeg,
        DriveConstants.kFRDriveAbsoluteEncoderReversed
    );

    private final SwerveModule BL = new SwerveModule(
        DriveConstants.kBLDriveMotorPort,
        DriveConstants.kBLTurningMotorPort,
        DriveConstants.kBLDriveEncoderReversed,
        DriveConstants.kBLTurningEncoderReversed,
        DriveConstants.kBLDriveAbsoluteEncoderPort,
        DriveConstants.kBLDriveAbsoluteEncoderOffsetDeg,
        DriveConstants.kBLDriveAbsoluteEncoderReversed
    );

    private final SwerveModule BR = new SwerveModule(
        DriveConstants.kBRDriveMotorPort,
        DriveConstants.kBRTurningMotorPort,
        DriveConstants.kBRDriveEncoderReversed,
        DriveConstants.kBRTurningEncoderReversed,
        DriveConstants.kBRDriveAbsoluteEncoderPort,
        DriveConstants.kBRDriveAbsoluteEncoderOffsetDeg,
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
          
              builder.addDoubleProperty("Front Left Angle", () -> Math.toDegrees(FR.getTurningPosition())-90, null);
              builder.addDoubleProperty("Front Left Velocity", () -> FR.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Front Right Angle", () -> Math.toDegrees(BR.getTurningPosition())-90, null);
              builder.addDoubleProperty("Front Right Velocity", () -> BR.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Back Left Angle", () -> Math.toDegrees(FL.getTurningPosition())-90, null);
              builder.addDoubleProperty("Back Left Velocity", () -> FL.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Back Right Angle", () -> Math.toDegrees(BL.getTurningPosition())-90, null);
              builder.addDoubleProperty("Back Right Velocity", () -> BL.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Robot Angle", () -> getHeading(), null);
            }
          });

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.kPXYController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                        AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                        0.3996, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void zeroHeading() {
        gyro.reset();
        FL.resetEncoders();
        FR.resetEncoders();
        BL.resetEncoders();
        BR.resetEncoders();
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
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
      };
    }

    public void stopModules() {
        FL.stop();
        FR.stop();
        BL.stop();
        BR.stop();
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        FL.setDesiredState(desiredStates[0]);
        FR.setDesiredState(desiredStates[1]);
        BL.setDesiredState(desiredStates[2]);
        BR.setDesiredState(desiredStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        FL.setDesiredState(desiredStates[0]);
        FR.setDesiredState(desiredStates[1]);
        BL.setDesiredState(desiredStates[2]);
        BR.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            FL.getState(),
            FR.getState(),
            BL.getState(),
            BR.getState()
        };
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        field.setRobotPose(new Pose2d(getPose().getY(), getPose().getX(), getPose().getRotation()));
    }

    public void switchIdleMode(){
        FL.switchIdleMode();
        FR.switchIdleMode();
        BL.switchIdleMode();
        BR.switchIdleMode();
    }
}