// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PolarCoordinate;
import frc.robot.util.fieldmirroring.FlippableBlueAlliancePose;

public class FieldOrientedVisionAlignPoint extends Command {

   private Swerve mSwerve;
    private DriverControls mDriverControls;
    private FlippableBlueAlliancePose mPoseToAlign;
  /** Creates a new FieldOrientedVisionAlign. */
  public FieldOrientedVisionAlignPoint(FlippableBlueAlliancePose pointToAlign){
        addRequirements(RobotContainer.S_SWERVE);
        mSwerve = RobotContainer.S_SWERVE;
        mDriverControls = RobotContainer.S_DRIVERCONTROLS;
        mPoseToAlign = pointToAlign;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSwerve.enableHeadingCorrection();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d difference = mPoseToAlign.getTranslation(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance()).minus(RobotContainer.S_SWERVE.getPose().getTranslation());
    mSwerve.driveFieldOriented(mSwerve.getTargetSpeedsFromPreScaledInputs(
                    mDriverControls.translationX(),
                    mDriverControls.translationY(),
                    Rotation2d.fromRadians(PolarCoordinate.toPolarCoordinate(() -> difference.getX(), () -> difference.getY())[1])));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.disableHeadingCorrection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Translation2d difference = mPoseToAlign.getTranslation(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance()).minus(RobotContainer.S_SWERVE.getPose().getTranslation());
    return Math.abs(PolarCoordinate.toPolarCoordinate(() -> difference.getX(), () -> difference.getY())[1] - mSwerve.getPose().getRotation().getRadians()) < Units.degreesToRadians(2.00);
  }
}
