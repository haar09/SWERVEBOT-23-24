package frc.robot.commands.AmpAutoShoot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPivot;

public class GoToAmp extends Command{
    private final ShooterPivot m_ShooterPivot;

    public GoToAmp(ShooterPivot shooter){
        this.m_ShooterPivot = shooter;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
            Pose2d targetPose = AutoConstants.kAmpPose;

            // Create the constraints to use while pathfinding
            PathConstraints constraints = AutoConstants.kPathConstraints;

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0, // Goal end velocity in meters/sec
                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );
            
            m_ShooterPivot.setDesiredAngle(ShooterConstants.kMaxShooterAngleRad);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}