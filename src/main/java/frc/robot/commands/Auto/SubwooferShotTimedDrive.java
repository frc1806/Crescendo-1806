// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Swerve.RobotOrientedTimedDrive;
import frc.robot.game.Shot;
import frc.robot.commands.sequence.PresetShotLaunchSequence;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubwooferShotTimedDrive extends SequentialCommandGroup {
  /** Creates a new SubwooferShotTimedDrive. */
  public SubwooferShotTimedDrive(Pose2d pose, Rotation2d directionToDrive) {

    addCommands( 
        RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
        RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
        RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
        RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
        new PresetShotLaunchSequence(Shot.SUBWOOFER), 
        new RobotOrientedTimedDrive(new Translation2d(0.5, directionToDrive), 5.0)
      );
      }
}
