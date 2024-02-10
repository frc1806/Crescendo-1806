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
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Launcher.SetLauncher;
import frc.robot.commands.Swerve.FieldOrientedDrive;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.sequence.IntakeSequence;
import frc.robot.commands.sequence.PresetShotLaunchSequence;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.BoatHook;
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
  public static final BoatHook S_BOATHOOK = new BoatHook();

  private static final VisionShotLibrary mVisionShotLibrary = new VisionShotLibrary() //these are estimates TODO: Tune
    .withShotEntry(3, 250.0, 3800.0)
    .withShotEntry(9, 225.0, 6000.0); 

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    setDefaultCommands();
    
    NamedCommands.registerCommand("lockPods", new LockPods(S_SWERVE));
    NamedCommands.registerCommand("resetGyro", new ResetGyro(S_SWERVE));
    NamedCommands.registerCommand("mobilityPointDrive", S_SWERVE.autonDriveBackwards(5));
    NamedCommands.registerCommand("intakeMax", new SetIntake(1.0));
    NamedCommands.registerCommand("intakeStop", new SetIntake(0.0));
    NamedCommands.registerCommand("visionShot", new VisionShot(S_ANGLER, S_LAUNCHER, S_DRIVERCONTROLS, S_LED, mVisionShotLibrary, 5.0));
    NamedCommands.registerCommand("IntakeSequence", new IntakeSequence());
    NamedCommands.registerCommand("PresetSubwooferShot", new PresetShotLaunchSequence(Shot.SUBWOOFER));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);


  }

  private void setDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(S_SWERVE, new FieldOrientedDrive(S_SWERVE, S_DRIVERCONTROLS));
    CommandScheduler.getInstance().setDefaultCommand(S_INTAKE, new SetIntake(0.0));
    CommandScheduler.getInstance().setDefaultCommand(S_ANGLER, new AnglerGoToAngle(Shot.HOME.getPivotAngle()));
    CommandScheduler.getInstance().setDefaultCommand(S_LAUNCHER, new SetLauncher(0.0));
  }

  private void configureBindings() {

    S_DRIVERCONTROLS.registerTriggers(S_SWERVE, S_INTAKE, S_ANGLER, S_VISION, S_LAUNCHER, S_LED, S_BOATHOOK, mVisionShotLibrary);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
