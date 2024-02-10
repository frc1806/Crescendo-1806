// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Launcher.LauncherIntake;
import frc.robot.game.Shot;

public class IntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  public IntakeSequence() {

    addCommands(
      new ParallelRaceGroup(
        new AnglerGoToAngle(Shot.HOME.getPivotAngle()),
        new LauncherIntake()
      ),
      new ParallelCommandGroup(
        new AnglerGoToAngle(Shot.HOME.getPivotAngle()),
        new ParallelRaceGroup(
          new LauncherIntake(),
          new SetIntake(1.0)
        )
      )
    );
  }
}
