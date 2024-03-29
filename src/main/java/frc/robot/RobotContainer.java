// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.sequence.VisionShotSequence;
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Auto.SubwooferShotTimedDrive;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Launcher.IdleLauncher;
import frc.robot.commands.Swerve.FieldOrientedDrive;
import frc.robot.commands.Swerve.FieldOrientedVisionAlignPoint;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.sequence.IntakeSequence;
import frc.robot.commands.sequence.PreparePresetLaunchSequence;
import frc.robot.commands.sequence.PresetShotLaunchSequence;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.BoatHook;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.DriverStationChecker;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Reel;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.fieldmirroring.FlippableBlueAlliancePose;
import frc.robot.subsystems.Launcher;

public class RobotContainer {

    private static final VisionShotLibrary VISION_SHOT_LIBRARY = new VisionShotLibrary() //these are estimates TODO: Tune
    .withShotEntry(1.09, 335.0, 5000.0)
    .withShotEntry(1.68, 325.0, 5000.0)
    .withShotEntry(2.14, 321.0, 5000.0)
    
    .withShotEntry(2.66, 319.0, 5000.0)
    .withShotEntry(3.14, 315.25, 5500.0
    );
  public static final Swerve S_SWERVE = new Swerve();
  public static final DriverControls S_DRIVERCONTROLS = new DriverControls();
  public static final Reel S_INTAKE = new Reel();
  public static final Angler S_ANGLER = new Angler();
  public static final Vision S_VISION = new Vision(VISION_SHOT_LIBRARY);
  public static final Launcher S_LAUNCHER = new Launcher();
  public static final LED S_LED = new LED();
  public static final BoatHook S_BOATHOOK = new BoatHook();
  public static final DriverStationChecker S_DRIVERSTATIONCHECKER = new DriverStationChecker();





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
    NamedCommands.registerCommand("visionShot", new VisionShotSequence(S_VISION::CalculateShotAngle, S_VISION::CalculateShotSpeed));
    NamedCommands.registerCommand("IntakeSequence", new IntakeSequence());
    NamedCommands.registerCommand("PresetSubwooferShot", new PresetShotLaunchSequence(Shot.SUBWOOFER));
    NamedCommands.registerCommand("PreparePresetSubwooferShot", new PreparePresetLaunchSequence(Shot.SUBWOOFER));
    NamedCommands.registerCommand("PresetAmpShot", new PresetShotLaunchSequence(Shot.AMPLIFIER));
    NamedCommands.registerCommand("turnTowardsCloseAmpNote", new FieldOrientedVisionAlignPoint(new FlippableBlueAlliancePose(new Translation2d(2.88, 6.99), Rotation2d.fromDegrees(0))));
    NamedCommands.registerCommand("turnTowardsCloseMidNote", new FieldOrientedVisionAlignPoint(new FlippableBlueAlliancePose(new Translation2d(2.88, 5.55), Rotation2d.fromDegrees(0))));
    NamedCommands.registerCommand("turnTowardsCloseSourceNote", new FieldOrientedVisionAlignPoint(new FlippableBlueAlliancePose(new Translation2d(2.88, 4.10), Rotation2d.fromDegrees(0))));
    NamedCommands.registerCommand("AnglerHome", new AnglerGoToAngle(Shot.HOME.getPivotAngle()));
    autoChooser = AutoBuilder.buildAutoChooser(); 

    //SUBWOOFER SINGLE SHOT MOBILITY AUTOS
    autoChooser.addOption("TimedSubwooferShot+MobilityBlueCenter", new SubwooferShotTimedDrive(new Pose2d(1.34, 5.52, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0.0)));
    autoChooser.addOption("TimedSubwooferShot+MobilityBlueDriversRight", new SubwooferShotTimedDrive(new Pose2d(0.73, 4.41, Rotation2d.fromDegrees(120)), Rotation2d.fromDegrees(0.0)));
    autoChooser.addOption("TimedSubwoofer+MobilityBlueDriversLeft", new SubwooferShotTimedDrive(new Pose2d(0.73, 6.66, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(120.0 + 180.0)));
    autoChooser.addOption("TimedSubwooferShot+MobilityRedCenter", new SubwooferShotTimedDrive(new Pose2d(15.19, 5.52, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0.0)));
    autoChooser.addOption("TimedSubwooferShot+MobilityRedDriversRight", new SubwooferShotTimedDrive(new Pose2d(15.82, 6.66, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-120.0 + 180.0)));
    autoChooser.addOption("TimedSubwoofer+MobilityRedDriversLeft", new SubwooferShotTimedDrive(new Pose2d(15.82, 4.41, Rotation2d.fromDegrees(60.0)), Rotation2d.fromDegrees(0.0)));

    autoChooser.addOption("SandstormModeForTesting", new FieldOrientedDrive());
    SmartDashboard.putData("Auto Mode", autoChooser);


  }

  private void setDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(S_SWERVE, new FieldOrientedDrive());
    CommandScheduler.getInstance().setDefaultCommand(S_INTAKE, new SetIntake(0.0));
    CommandScheduler.getInstance().setDefaultCommand(S_ANGLER, new AnglerGoToAngle(Shot.HOME.getPivotAngle()));
    CommandScheduler.getInstance().setDefaultCommand(S_LAUNCHER, new IdleLauncher());
  }

  private void configureBindings() {

    S_DRIVERCONTROLS.registerTriggers(S_SWERVE, S_INTAKE, S_ANGLER, S_VISION, S_LAUNCHER, S_LED, S_BOATHOOK, VISION_SHOT_LIBRARY);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
