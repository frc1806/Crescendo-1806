// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Angler.AnglerGoToAngleFromSupplier;
import frc.robot.commands.Launcher.LaunchNoteFromSupplier;
import frc.robot.commands.Launcher.SetLauncher;
import frc.robot.commands.Launcher.SetLauncherFromSupplier;
import frc.robot.commands.Swerve.FieldOrientedDrive;
import frc.robot.commands.Swerve.FieldOrientedVisionAlignSpeaker;
import frc.robot.game.Shot;
import java.util.function.DoubleSupplier;


public class VisionShotSequence extends SequentialCommandGroup {

  /** Creates a new LaunchSequence. */
  public VisionShotSequence(DoubleSupplier angleSupplier, DoubleSupplier speedSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
                new ParallelRaceGroup(
                  new ParallelCommandGroup(
                    new SetLauncherFromSupplier(speedSupplier), 
                      new AnglerGoToAngleFromSupplier(angleSupplier)
                  ),
                  new FieldOrientedVisionAlignSpeaker()
                ),
                new ParallelRaceGroup(
                  new ParallelCommandGroup(                 
                    new LaunchNoteFromSupplier(speedSupplier), 
                      new AnglerGoToAngleFromSupplier(angleSupplier)
                  ),
                  new FieldOrientedVisionAlignSpeaker()

                )
    );
  }
}
