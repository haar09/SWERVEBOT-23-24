package frc.robot.commands.TeleopPath;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.GlobalVariables;

public class GoToAmp extends Command{
    private Command pathCommand;
    private Pose2d targetPose;

    public GoToAmp(){
    }

    @Override
    public void initialize(){
        if (GlobalVariables.getInstance().alliance == Alliance.Red) {
            targetPose = new Pose2d(14.7, 8.1, Rotation2d.fromDegrees(90));
        } else {
            targetPose = new Pose2d(1.84, 8.1, Rotation2d.fromDegrees(90));
        }
            // Create the constraints to use pathfinding
            PathConstraints constraints = AutoConstants.kPathConstraints;

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            pathCommand = AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0, // Goal end velocity in meters/sec
                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );
    }

    @Override
    public void execute(){        
            pathCommand.schedule();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}