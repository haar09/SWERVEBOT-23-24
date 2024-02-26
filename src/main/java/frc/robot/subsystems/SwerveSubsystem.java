package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

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
    private final SwerveDrivePoseEstimator poseEstimator;
    private final LimeLight limelight;

    public SwerveSubsystem(LimeLight m_limeLight) {
        new Thread (() -> {
            while (true) {
                if (gyro.isConnected()) {
                    try {
                        Thread.sleep(1000);
                        gyro.reset();
                        break;
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
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
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        limelight = m_limeLight;


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
                        new Translation2d(DriveConstants.kOnArkaArasi / 2, DriveConstants.kSagSolArasi / 2).getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
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
        System.out.println("RESETTED!");
    }

    public double getHeading() {
        return gyro.getYaw() * -1;
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
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = limelight.getEstimationStdDevs(estPose);
    
                    addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

        
        poseEstimator.update(getRotation2d(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        
        
        SmartDashboard.putNumber("FL", FL.getAmperage());
        SmartDashboard.putNumber("FR", FR.getAmperage());
        SmartDashboard.putNumber("BL", BL.getAmperage());
        SmartDashboard.putNumber("BR", BR.getAmperage());        

        if (resetNavXEntry.getBoolean(false)) {
            zeroHeading();
            resetNavXEntry.setBoolean(false);
        }
    }

    public void switchIdleMode(){
        FL.switchIdleMode();
        FR.switchIdleMode();
        BL.switchIdleMode();
        BR.switchIdleMode();
    }

    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public Command goTo(){
            // Since we are using a holonomic drivetrain, the rotation component of this pose
            // represents the goal holonomic rotation
            Pose2d targetPose = new Pose2d(1.9, 7.5, Rotation2d.fromDegrees(90));

            // Create the constraints to use while pathfinding
            PathConstraints constraints = new PathConstraints(
                    3.0, 3.0,
                    Math.toRadians(540), Math.toRadians(720));

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            Command pathfindingCommand = AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0, // Goal end velocity in meters/sec
                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );

            return pathfindingCommand;
    }

}