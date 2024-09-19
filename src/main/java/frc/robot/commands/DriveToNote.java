package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToNote extends Command{
        
    private double angle;
    private ObjectDetection detector;
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController driverJoystick;
    private final Intake intake;
    private final Extender extender;
    private final LEDSubsystem ledSubsystem;

    PIDController thetaController = new PIDController(PIDConstants.kPObjectRotate, 0, PIDConstants.kDObjectRotate);

    public DriveToNote(ObjectDetection objectDetector, SwerveSubsystem swerveSubsystem, XboxController driverController,
    Intake intake, Extender extender, LEDSubsystem ledSubsystem){
        this.detector = objectDetector;
        this.swerveSubsystem = swerveSubsystem;
        this.driverJoystick = driverController;
        this.intake = intake;
        this.extender = extender;
        this.ledSubsystem = ledSubsystem;
        thetaController.enableContinuousInput(0, 360);
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        ledSubsystem.isAutodrive = true;
        
        angle = detector.getTX();

        if (!GlobalVariables.getInstance().extenderFull) {
            intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
            extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
        } else {
            driverJoystick.setRumble(RumbleType.kBothRumble, 0.6);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(-2.5, 0, thetaController.calculate(angle))
        );

        swerveSubsystem.setModuleStates(moduleStates);
    }

    public void end(boolean interrupted){
        ledSubsystem.isAutodrive = false;
        GlobalVariables.getInstance().customRotateSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}