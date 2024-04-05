// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Angler.AnglerGoToAngleFromSupplier;
import frc.robot.commands.Launcher.SetLauncherFromSupplier;
import frc.robot.commands.Swerve.FieldOrientedVisionAlignSpeaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareVisionShotNoAlign extends SequentialCommandGroup {
  /** Creates a new PrepareVisionShotNoAlign. */
  public PrepareVisionShotNoAlign(DoubleSupplier angleSupplier, DoubleSupplier speedSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
                  new ParallelCommandGroup(
                    new SetLauncherFromSupplier(speedSupplier), 
                      new AnglerGoToAngleFromSupplier(angleSupplier)
                  )
    ;
  }
}
