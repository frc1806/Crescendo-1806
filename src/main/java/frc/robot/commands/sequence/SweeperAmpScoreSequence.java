// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Sweeper.SweeperAmpIntake;
import frc.robot.commands.Sweeper.SweeperAmpOuttake;
import frc.robot.commands.Sweeper.SweeperExtend;
import frc.robot.game.SweeperPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SweeperAmpScoreSequence extends SequentialCommandGroup {
  /** Creates a new SweeperAmpScoreSequence. */
  public SweeperAmpScoreSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SweeperExtend(0),
      new AnglerGoToAngle(SweeperPosition.INTAKE.getAngle()),
      new SweeperExtend(SweeperPosition.INTAKE.getDistance()),
      new SweeperAmpOuttake()
    );
  }
}
