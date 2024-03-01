// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;

public class IdleLauncher extends Command {
  private Launcher mLauncher;
  /** Creates a new IdleLauncher. */
  public IdleLauncher() {
    mLauncher = RobotContainer.S_LAUNCHER;
    addRequirements(RobotContainer.S_LAUNCHER);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    /* TODO: Maybe someday?
    if(RobotContainer.S_DRIVERSTATIONCHECKER.isRealMatch()){
      if(mLauncher.isNoteInIndexer()){
        mLauncher.setLauncher(3750);
      }
      else{
        mLauncher.setLauncher(-1500);
      }
    }
    else{
        mLauncher.setLauncher(0);
    }
    mLauncher.stopIndexer();
    */
    mLauncher.setLauncher(0);
    mLauncher.stopIndexer();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
