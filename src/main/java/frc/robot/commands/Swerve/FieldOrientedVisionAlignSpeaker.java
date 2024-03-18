// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Swerve;

public class FieldOrientedVisionAlignSpeaker extends Command {

   private Swerve mSwerve;
    private DriverControls mDriverControls;
  /** Creates a new FieldOrientedVisionAlign. */
  public FieldOrientedVisionAlignSpeaker(){
        addRequirements(RobotContainer.S_SWERVE);
        mSwerve = RobotContainer.S_SWERVE;
        mDriverControls = RobotContainer.S_DRIVERCONTROLS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSwerve.enableHeadingCorrection();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSwerve.driveFieldOriented(mSwerve.getTargetSpeedsFromPreScaledInputs(
                    mDriverControls.translationX(),
                    mDriverControls.translationY(),
                    RobotContainer.S_VISION.getYawToSpeaker()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.disableHeadingCorrection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
