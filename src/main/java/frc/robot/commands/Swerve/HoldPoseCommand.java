// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class HoldPoseCommand extends Command {
 
  protected Pose2d mPoseToHold;
  protected Swerve mSwerve;
  private PIDController xPid;
  private PIDController yPid;


  public HoldPoseCommand(Pose2d pose) {
    mSwerve = RobotContainer.S_SWERVE;
    addRequirements(RobotContainer.S_SWERVE);
    xPid = new PIDController(Constants.kSwerveAutoPIDP, Constants.kSwerveAutoPIDI, Constants.kSwerveAutoPIDD);
    yPid = new PIDController(Constants.kSwerveAutoPIDP, Constants.kSwerveAutoPIDI, Constants.kSwerveAutoPIDD);
    mPoseToHold = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.S_DRIVERCONTROLS.setLastSnapDegree(mPoseToHold.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSwerve.driveFieldOriented(mSwerve.getTargetSpeedsFromPreScaledInputs(
                xPid.calculate(mSwerve.getPose().getX(), mPoseToHold.getX()),
                yPid.calculate(mSwerve.getPose().getY(), mPoseToHold.getY()),
                mPoseToHold.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
