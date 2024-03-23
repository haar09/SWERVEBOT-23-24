// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Degree180Turn;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.RotateToTargetWhileDrive;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoCommands.IntakeIn;
//import frc.robot.commands.AutoCommands.LimeLightRotateToTarget;
import frc.robot.commands.AutoCommands.SpeakerShoot;
import frc.robot.commands.SetAngle.AmpAngle;
import frc.robot.commands.SetAngle.ShooterAngle;
import frc.robot.commands.SetAngle.ShooterAutoAngle;
import frc.robot.commands.ShootCommands.AmpShoot;
import frc.robot.commands.ShootCommands.ShooterShoot;
import frc.robot.commands.TeleopPath.GoToAmp;
import frc.robot.commands.TeleopPath.GoToSource;
import frc.robot.commands.TeleopPath.GoToSpeaker;
import frc.robot.commands.SetAngle.Shooter1mAngle;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterPivot shooterPivot = new ShooterPivot();
  private final LimeLight LimeLight;
  private final LEDSubsystem ledSubsystem;
  private final Extender extender;
  private final Intake intake;
  private final Shooter shooter;
  private final ColorSensor colorSensor;

  public static final PS4Controller driverJoystick = new PS4Controller(0);
  public static final PS4Controller operatorJoystick = new PS4Controller(1);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    LimeLight = new LimeLight();
    swerveSubsystem = new SwerveSubsystem(LimeLight);
    ledSubsystem = new LEDSubsystem();
    extender = new Extender();
    intake = new Intake();
    colorSensor = new ColorSensor();
    shooter = new Shooter(ledSubsystem);

    intake.setDefaultCommand(new IntakeCmd(        
      () -> driverJoystick.getRawButton(8),
      () -> driverJoystick.getRawButton(7),
      () -> driverJoystick.getRawButton(2),
      intake,
      extender
    ));
    
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoystick.getRawAxis(1),
        () -> -driverJoystick.getRawAxis(0),
        () -> -driverJoystick.getRawAxis(2), 
        () -> !driverJoystick.getRawButton(5), //bu L1
        () -> !driverJoystick.getRawButton(6), // bu R1
        () -> driverJoystick.getRawButton(14)
      )
    );
    
    //NamedCommands.registerCommand("RotateToShooter", new LimeLightRotateToTarget(LimeLight, swerveSubsystem));

    NamedCommands.registerCommand("ShootToSpeakerAuto", new SpeakerShoot(shooter, shooterPivot, extender, ledSubsystem, 0));
    NamedCommands.registerCommand("ShootToSpeaker0m", new SpeakerShoot(shooter, shooterPivot, extender, ledSubsystem, ShooterConstants.k0mAngle));
    NamedCommands.registerCommand("ShootToSpeaker1m", new SpeakerShoot(shooter, shooterPivot, extender, ledSubsystem, ShooterConstants.k1mAngle));
    NamedCommands.registerCommand("ShootToSpeaker15m", new SpeakerShoot(shooter, shooterPivot, extender, ledSubsystem, ShooterConstants.k15mAngle));
    NamedCommands.registerCommand("ShootToSpeaker2m", new SpeakerShoot(shooter, shooterPivot, extender, ledSubsystem, ShooterConstants.k2mAngle));
    NamedCommands.registerCommand("ShootToSpeaker25m", new SpeakerShoot(shooter, shooterPivot, extender, ledSubsystem, ShooterConstants.k25mAngle));

    NamedCommands.registerCommand("IntakeIn", new IntakeIn(intake, extender, colorSensor));
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 13).onTrue(new InstantCommand(swerveSubsystem::zeroHeading)); //ps butonu

    new POVButton(driverJoystick, 0).whileTrue(new GoToSpeaker()); // yukarı ok
    new POVButton(driverJoystick, 180).whileTrue(new GoToSource()); // aşağı ok
    new POVButton(driverJoystick, 90).whileTrue(new GoToAmp()); // sağ ok
    new POVButton(driverJoystick, 270).whileTrue(new GoToAmp()); // sol ok

    new JoystickButton(operatorJoystick, 6).whileTrue(new ShooterShoot(
      () -> operatorJoystick.getRawAxis(4),
      shooter, intake, extender)); //r1 ve r2

    new JoystickButton(operatorJoystick, 5).whileTrue(new AmpShoot(
      () -> operatorJoystick.getRawAxis(4),
      shooter, intake, extender)); //r1 ve r2
    
    
    new JoystickButton(operatorJoystick, 1).whileTrue(new AmpAngle(shooterPivot)); // kare
    new JoystickButton(operatorJoystick, 2).whileTrue(new ShooterAngle(shooterPivot)); // x
    new JoystickButton(operatorJoystick, 3).whileTrue(new Shooter1mAngle(shooterPivot)); // daire
    new JoystickButton(operatorJoystick, 4).whileTrue(new ShooterAutoAngle(shooterPivot)); // üçgen
    
    new JoystickButton(driverJoystick, 3).whileTrue(new RotateToTargetWhileDrive(swerveSubsystem)); // bu daire
    new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(swerveSubsystem::switchIdleMode)); // bu kare
    new JoystickButton(driverJoystick, 4).onTrue(new Degree180Turn(swerveSubsystem)); //üçgen
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
