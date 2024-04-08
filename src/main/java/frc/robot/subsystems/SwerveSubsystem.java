package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveModule FL = new SwerveModule(
        DriveConstants.kFLDriveMotorPort,
        DriveConstants.kFLTurningMotorPort,
        DriveConstants.kFLDriveEncoderReversed,
        DriveConstants.kFLTurningEncoderReversed,
        DriveConstants.kFLTurningAbsoluteEncoderPort,
        DriveConstants.kFLTurningAbsoluteEncoderOffset
    );

    private final SwerveModule FR = new SwerveModule(
        DriveConstants.kFRDriveMotorPort,
        DriveConstants.kFRTurningMotorPort,
        DriveConstants.kFRDriveEncoderReversed,
        DriveConstants.kFRTurningEncoderReversed,
        DriveConstants.kFRTurningAbsoluteEncoderPort,
        DriveConstants.kFRTurningAbsoluteEncoderOffset
    );

    private final SwerveModule BL = new SwerveModule(
        DriveConstants.kBLDriveMotorPort,
        DriveConstants.kBLTurningMotorPort,
        DriveConstants.kBLDriveEncoderReversed,
        DriveConstants.kBLTurningEncoderReversed,
        DriveConstants.kBLTurningAbsoluteEncoderPort,
        DriveConstants.kBLTurningAbsoluteEncoderOffset
    );

    private final SwerveModule BR = new SwerveModule(
        DriveConstants.kBRDriveMotorPort,
        DriveConstants.kBRTurningMotorPort,
        DriveConstants.kBRDriveEncoderReversed,
        DriveConstants.kBRTurningEncoderReversed,
        DriveConstants.kBRTurningAbsoluteEncoderPort,
        DriveConstants.kBRTurningAbsoluteEncoderOffset
    );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Field2d field = new Field2d();
    private GenericEntry resetNavXEntry;
    public final SwerveDrivePoseEstimator poseEstimator;
    private final LimeLight limelight;
    private final OV9281 ov9281;
    public double speakerDistance;
    public Translation2d kSpeakerApriltagPose;

    public SwerveSubsystem(LimeLight m_limeLight, OV9281 m_camera) {
        new Thread (() -> {
            while (true) {
                if (gyro.isConnected()) {
                    try {
                        Thread.sleep(1000);
                        gyro.reset();
                        break;
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }).start();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Gyro", gyro);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
              builder.setSmartDashboardType("SwerveDrive");
          
              builder.addDoubleProperty("Front Left Angle", () -> Math.toDegrees(FL.getTurningPosition()), null);
              builder.addDoubleProperty("Front Left Velocity", () -> FL.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Front Right Angle", () -> Math.toDegrees(FR.getTurningPosition()), null);
              builder.addDoubleProperty("Front Right Velocity", () -> FR.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Back Left Angle", () -> Math.toDegrees(BL.getTurningPosition()), null);
              builder.addDoubleProperty("Back Left Velocity", () -> BL.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Back Right Angle", () -> Math.toDegrees(BR.getTurningPosition()), null);
              builder.addDoubleProperty("Back Right Velocity", () -> BR.getDriveVelocity(), null);
          
              builder.addDoubleProperty("Robot Angle", () -> getHeading(), null);
            }
          });
          
        resetNavXEntry = Shuffleboard.getTab("TabName")
        .add("Reset NavX", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

        //pose estimation
        var stateStdDevs = VecBuilder.fill(0.05, 0.05, Math.toRadians(5));
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        limelight = m_limeLight;
        ov9281 = m_camera;

        resetEncoders();

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
                        new Translation2d(0.259428, DriveConstants.kSagSolArasi / 2).getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
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

    private void resetEncoders() {
        FL.resetEncoders();
        FR.resetEncoders();
        BL.resetEncoders();
        BR.resetEncoders();
    }

    public void zeroHeading() {
        gyroOffset = 0;
        gyro.reset();
        resetEncoders();
        System.out.println("RESETTED!");
    }

    public double gyroOffset = 0;

    public double getHeading() {
        return gyro.getYaw() * -1 ;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition(),
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
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void resetOdometry(Pose2d pose) {
        zeroHeading();
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        var visionEst = limelight.getEstimatedGlobalPose();
        if (limelight.isTargetValid()){
            addVisionMeasurement(
                            visionEst.pose, visionEst.timestampSeconds, VisionConstants.kLimelightStdDevs);
        };

        var visionEst1 = ov9281.getEstimatedGlobalPose1();
        visionEst1.ifPresent(
                est1 -> {
                    var estPose1 = est1.estimatedPose.toPose2d();
                    var estStdDevs1 = ov9281.getEstimationStdDevs1(estPose1);
    
                    addVisionMeasurement(
                            est1.estimatedPose.toPose2d(), est1.timestampSeconds, estStdDevs1);
        });

        var visionEst2 = ov9281.getEstimatedGlobalPose2();
        visionEst2.ifPresent(
                est2 -> {
                    var estPose2 = est2.estimatedPose.toPose2d();
                    var estStdDevs2 = ov9281.getEstimationStdDevs2(estPose2);
    
                    addVisionMeasurement(
                            est2.estimatedPose.toPose2d(), est2.timestampSeconds, estStdDevs2);
        });

        poseEstimator.update(getRotation2d(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());     

        if (resetNavXEntry.getBoolean(false)) {
            zeroHeading();
            resetNavXEntry.setBoolean(false);
        }

        if (GlobalVariables.getInstance().alliance != null) {
            if (GlobalVariables.getInstance().alliance == Alliance.Red) {
                kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(4).get().getTranslation().toTranslation2d();
            } else {
                kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(7).get().getTranslation().toTranslation2d();
            }
        } else {
                kSpeakerApriltagPose = new Translation2d(1000,1000);
        }

        speakerDistance = kSpeakerApriltagPose.minus(getPose().getTranslation()).getNorm();

        GlobalVariables.getInstance().speakerDistance = speakerDistance;
        SmartDashboard.putNumber("speakerDistance", speakerDistance);    
    }

    public void switchIdleMode(){
        FL.switchIdleMode();
        FR.switchIdleMode();
        BL.switchIdleMode();
        BR.switchIdleMode();
    }

    public void offsetGyro(){
        System.out.println(getPose().getRotation().getDegrees());
        gyroOffset = getPose().getRotation().getDegrees();
    }

    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
}