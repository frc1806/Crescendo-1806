// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;

public class GetToPoseCommand extends HoldPoseCommand {
  /** Creates a new GetToPoseCommand. */
  public GetToPoseCommand(Pose2d pose) {
    super(pose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(mPoseToHold.getX() - mSwerve.getPose().getX()) < 0.01 && 
            Math.abs(mPoseToHold.getY() - mSwerve.getPose().getY()) < 0.01 &&
            Math.abs(mSwerve.getPose().getRotation().minus(mPoseToHold.getRotation()).getDegrees()) < 1.0;
  }
}
