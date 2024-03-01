// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;

public class LauncherOuttake extends Command {

  Launcher mLauncher;
  /** Creates a new LauncherOuttake. */
  public LauncherOuttake() {
    mLauncher = RobotContainer.S_LAUNCHER;
    addRequirements(mLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLauncher.runMotorsForOuttake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mLauncher.runMotorsForOuttake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLauncher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mLauncher.isNoteInIndexer();
  }
}