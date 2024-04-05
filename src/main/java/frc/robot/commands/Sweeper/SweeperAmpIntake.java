// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sweeper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SweeperAmpIntake extends Command {
  /** Creates a new SweeperAmpIntake. */
  public SweeperAmpIntake() {
    addRequirements(RobotContainer.S_SWEEPER_INTAKE);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.S_SWEEPER_INTAKE.intakeSweep();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.S_SWEEPER_INTAKE.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
