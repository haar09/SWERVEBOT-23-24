// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LimeLightLEDToggle;
import frc.robot.commands.LimeLightRotateToTarget;
import frc.robot.commands.RotateToTargetWhileDrive;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem;
  private final LimeLight LimeLight = new LimeLight();
  private final Joystick driverJoystick = new Joystick(0);
    
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoystick.getRawAxis(1),
        () -> -driverJoystick.getRawAxis(0),
        () -> -driverJoystick.getRawAxis(2), 
        () -> !driverJoystick.getRawButton(5), //bu L1
        () -> !driverJoystick.getRawButton(6) // bu R1
      )
    );
    
    NamedCommands.registerCommand("RotateToShooter", new LimeLightRotateToTarget(LimeLight, swerveSubsystem));
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 13).onTrue(new InstantCommand(swerveSubsystem::zeroHeading)); //ps butonu
    new JoystickButton(driverJoystick, 10).onTrue(new LimeLightLEDToggle(LimeLight)); //options

    //new JoystickButton(driverJoystick, 4).whileTrue(new LimeLightFollowReflector(LimeLight, swerveSubsystem, 0)); // üçgen
    //new JoystickButton(driverJoystick, 11).whileTrue(new LimeLightFollowReflector(LimeLight, swerveSubsystem, 1)); // L3
    
    new JoystickButton(driverJoystick, 3).whileTrue(new RotateToTargetWhileDrive(LimeLight)); // bu daire
    new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(swerveSubsystem::switchIdleMode)); // bu kare
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
