// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;

public class SetLauncherFromSupplier extends Command {
  /** Creates a new SetLauncherFromVision. */
  private DoubleSupplier mSpeedDoubleSupplier;
  private Launcher mLauncher;

  public SetLauncherFromSupplier(DoubleSupplier speedSupplier) {
     mLauncher = RobotContainer.S_LAUNCHER;
     mSpeedDoubleSupplier = speedSupplier;
     addRequirements(mLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLauncher.setLauncher(mSpeedDoubleSupplier.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mLauncher.setLauncher(mSpeedDoubleSupplier.getAsDouble());

    mLauncher.stopIndexer(); //idle the indexer while spinning up flywheel... or stopping flywheel.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mLauncher.isLauncherAtSpeed();
  }
}
