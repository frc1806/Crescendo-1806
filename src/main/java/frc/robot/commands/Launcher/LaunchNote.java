// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;

public class LaunchNote extends Command {
  private Launcher mLauncher;
  private double mSpeed;
  private boolean mIsNoteLeaving = false;
  private static final double mTimeout = 5.0;
  private Timer mTimer = new Timer();

  /** Creates a new LaunchNote. */
  public LaunchNote(double wantedSpeed) {
    mLauncher = RobotContainer.S_LAUNCHER;
    addRequirements(RobotContainer.S_LAUNCHER);
    mSpeed = wantedSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLauncher.setLauncher(mSpeed);
    mLauncher.sendNoteToFlywheel();
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mLauncher.isNoteAtEndOfLauncher()){
      mIsNoteLeaving = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLauncher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (mIsNoteLeaving && mLauncher.isLauncherAndIndexerEmpty()) || mTimer.hasElapsed(mTimeout);
  }
}
