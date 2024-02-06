// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.VisionShot;
import frc.robot.commands.Swerve.FieldOrientedDrive;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Reel;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
  
  public static final Swerve S_SWERVE = new Swerve();
  public static final DriverControls S_DRIVERCONTROLS = new DriverControls();
  public static final Reel S_INTAKE = new Reel();
  public static final Angler S_ANGLER = new Angler();
  public static final Vision S_VISION = new Vision();
  public static final Launcher S_LAUNCHER = new Launcher();
  public static final LED S_LED = new LED();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    setDefaultCommands();
    
    NamedCommands.registerCommand("lockPods", new LockPods(S_SWERVE));
    NamedCommands.registerCommand("resetGyro", new ResetGyro(S_SWERVE));
    NamedCommands.registerCommand("intakeMax", S_INTAKE.setIntake(1.0));
    NamedCommands.registerCommand("intakeStop", S_INTAKE.setIntake(0));
    NamedCommands.registerCommand("visionShot", new VisionShot(S_ANGLER, S_LAUNCHER, S_DRIVERCONTROLS, S_LED));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void setDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(S_SWERVE, new FieldOrientedDrive(S_SWERVE, S_DRIVERCONTROLS));
  }

  private void configureBindings() {
    S_DRIVERCONTROLS.registerTriggers(S_SWERVE, S_INTAKE, S_ANGLER, S_VISION, S_LAUNCHER, S_LED);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
