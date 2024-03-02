// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RobotOrientedTimedDrive extends Command {

  Timer mTimer;
  Double mTimeToDrive;
  Translation2d mDirection;
  /** Creates a new RobotOrientedDrive. */
  public RobotOrientedTimedDrive(Translation2d direction, double timeToDrive) {
    addRequirements(RobotContainer.S_SWERVE);
    mTimer= new Timer();
    mDirection = direction;
    mTimeToDrive = timeToDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.S_SWERVE.drive(mDirection, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.S_SWERVE.drive(new Translation2d(), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mTimeToDrive);
  }
}
