// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Swerve.FieldOrientedDrive;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  
  public final Swerve S_SWERVE = new Swerve();
  public final DriverControls S_DRIVERCONTROLS = new DriverControls();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void setDefaultCommands(){
    CommandScheduler.getInstance().setDefaultCommand(S_SWERVE, new FieldOrientedDrive(S_SWERVE, S_DRIVERCONTROLS));
  }

  private void configureBindings() {
    S_DRIVERCONTROLS.registerTriggers(S_SWERVE);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
