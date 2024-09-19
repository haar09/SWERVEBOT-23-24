package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShooterAuto extends Command{
    private final Shooter shooter;
    private final Extender extender;
    private final Intake intake;
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterPivot shooterPivot;
    private boolean ending;
    private final XboxController operatorController;

    public ShooterAuto(Shooter shooter, Intake intake,Extender extender,
        SwerveSubsystem swerveSubsystem, ShooterPivot shooterPivot, XboxController operatorController){
        this.shooter = shooter;
        this.extender = extender;
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem;
        this.shooterPivot = shooterPivot;
        this.operatorController = operatorController;
        ending = false;
        thetaController.enableContinuousInput(0, 360);
        addRequirements(shooter, extender); 
    }

    private enum State {
        START,
        EXTEND,
        SHOOT,
        END
    }

    private State state = State.START;
    private double startTime;

    private double angle;
    private Translation2d robotToTarget, kSpeakerApriltagPose;
    
    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, PIDConstants.kDLimeLightRotate);

    @Override
    public void initialize(){
        ending = false;
        state = State.START;
        shooter.state = ShooterState.IDLE;

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(4).get().getTranslation().toTranslation2d();
        } else {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(7).get().getTranslation().toTranslation2d();
        }
    }

    @Override
    public void execute() {
        /* rotate to target */
        robotToTarget = kSpeakerApriltagPose.minus(swerveSubsystem.getPose().getTranslation());

        angle = robotToTarget.getAngle().getDegrees();

        thetaController.setSetpoint(angle);
        /* */

        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        if (GlobalVariables.getInstance().extenderFull) {
            if (GlobalVariables.getInstance().speakerDistance > 4.5) {
                shooter.setSlowSpeed();
            } else {
                GlobalVariables.getInstance().customRotateSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getDegrees());
                shooter.setSpeakerSpeed();
            }
        } else {
            operatorController.setRumble(RumbleType.kBothRumble, 1);
            return;
        }

        shooterPivot.setDesiredAngle(GlobalVariables.getInstance().speakerToAngle());

        switch (state) {
            case START:
                if (shooter.state == ShooterState.READY) {
                    if (GlobalVariables.getInstance().extenderFull) {
                        if (GlobalVariables.getInstance().speakerToAngle() > 0 && Math.abs(thetaController.getPositionError()) < 6 &&
                        swerveSubsystem.gyro.getVelocityX() < 0.2 && swerveSubsystem.gyro.getVelocityY() < 0.2) {
                            operatorController.setRumble(RumbleType.kBothRumble, 0);
                            startTime = Timer.getFPGATimestamp();
                            state = State.EXTEND;
                        }
                    } else {
                        operatorController.setRumble(RumbleType.kBothRumble, 1);
                    }
                }
                break;
            case EXTEND:
                if (timeElapsed < 0.1) {
                    extender.setOutputPercentage(-IntakextenderConstants.kExtenderBackSpeed);
                } else {
                    state = State.SHOOT;
                }
                break;
            case SHOOT:
                if (timeElapsed < 1.3) {
                    shooter.setSpeakerSpeed();
                    extender.setOutputPercentage(1);
                    intake.setOutputPercentage(0.6);
                } else {
                    state = State.END;
                }
                break;
            case END:
                ending = true;
                end(false);
                extender.setOutputPercentage(0);
                shooter.stopShooter();
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted){
        state = State.START;
        shooter.state = ShooterState.IDLE;
        intake.setOutputPercentage(0);
        extender.setOutputPercentage(0);
        shooter.stopShooter();    
        SmartDashboard.putBoolean("shooterReady", false);

        GlobalVariables.getInstance().customRotateSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
