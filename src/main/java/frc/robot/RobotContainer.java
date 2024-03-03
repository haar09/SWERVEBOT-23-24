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
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.LimeLightRotateToTarget;
import frc.robot.commands.RotateToTargetWhileDrive;
//import frc.robot.commands.ShootAndSetRPM;
//import frc.robot.commands.ShooterAutoAim;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AmpAutoShoot.GoToAmp;
import frc.robot.subsystems.LimeLight;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem;
  /*private final ShooterPivot shooterPivot = new ShooterPivot();*/
  private final LimeLight LimeLight;
  private final LEDSubsystem ledSubsystem;
  private final Extender extender;
  private final Intake intake;
  //private final Shooter shooter;

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
    new ColorSensor();
    //shooter = new Shooter(ledSubsystem);

    /*shooter.setDefaultCommand(new ShootAndSetRPM(
      () -> operatorJoystick.getRawAxis(4),
      () -> operatorJoystick.getRawButton(4),
      shooter, extender, ledSubsystem));*/

    intake.setDefaultCommand(new IntakeCmd(        
      () -> -driverJoystick.getRawAxis(3),
      () -> -driverJoystick.getRawAxis(4),
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

    new JoystickButton(driverJoystick, 4).whileTrue(new GoToAmp()); // üçgen
    /*new JoystickButton(operatorJoystick, 5).whileTrue(new ShooterAutoAim(shooter, LimeLight)); // opeartör L1*/
    
    new JoystickButton(driverJoystick, 3).whileTrue(new RotateToTargetWhileDrive(LimeLight)); // bu daire
    new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(swerveSubsystem::switchIdleMode)); // bu kare
  }

  public static void rumble(double rumbleValue, double rumbleValue2, double rumbleValue3, double rumbleValue4) {
    driverJoystick.setRumble(PS4Controller.RumbleType.kLeftRumble, rumbleValue);
    driverJoystick.setRumble(PS4Controller.RumbleType.kRightRumble, rumbleValue2);
    operatorJoystick.setRumble(PS4Controller.RumbleType.kLeftRumble, rumbleValue3);
    operatorJoystick.setRumble(PS4Controller.RumbleType.kRightRumble, rumbleValue4);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
