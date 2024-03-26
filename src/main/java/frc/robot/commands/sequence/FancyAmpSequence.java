// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Launcher.LaunchNote;
import frc.robot.commands.Launcher.LaunchNoteNoSpin;
import frc.robot.commands.Launcher.SetLauncher;
import frc.robot.game.Shot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FancyAmpSequence extends SequentialCommandGroup {
  /** Creates a new FancyAmpSequence. */
  public FancyAmpSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
                  new SetLauncher(
                    Shot.FANCY_AMP_PT_1.getLauncherSpeed()), 
                    new AnglerGoToAngle(Shot.FANCY_AMP_PT_1.getPivotAngle()
                    )
                ),
                new ParallelCommandGroup( 
                  new SequentialCommandGroup(
                    new ParallelCommandGroup(
                      new SetLauncher(Shot.FANCY_AMP_PT_2.getLauncherSpeed()),
                      new WaitCommand(0.15)
                    ),
                    new LaunchNoteNoSpin(Shot.FANCY_AMP_PT_2.getLauncherSpeed())
                  ), 
                  new AnglerGoToAngle(Shot.FANCY_AMP_PT_2.getPivotAngle()
)
                ),
                new ParallelRaceGroup(
                    new SetLauncher(0.0),
                    new AnglerGoToAngle(Shot.HOME.getPivotAngle())
                )
    );
  }
}
