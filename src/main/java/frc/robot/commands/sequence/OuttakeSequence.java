// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Angler.AnglerGoToAngle;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Launcher.IdleLauncher;
import frc.robot.commands.Launcher.LauncherIntake;
import frc.robot.commands.Launcher.LauncherOuttake;
import frc.robot.game.Shot;
import frc.robot.util.rumbleutil.RumbleCommand;
import frc.robot.util.rumbleutil.SineWave;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeSequence extends SequentialCommandGroup {
  /** Creates a new OuttakeSequence. */
  public OuttakeSequence() {

    addCommands(
      new ParallelRaceGroup(
        new AnglerGoToAngle(Shot.HOME.getPivotAngle()),
        new IdleLauncher(),
        new SetIntake(0.0)
      ),
      new ParallelCommandGroup(
        new AnglerGoToAngle(Shot.HOME.getPivotAngle()),
        new ParallelRaceGroup(
          new LauncherOuttake(),
          new SetIntake(-1.0)
        )
      ),
      RobotContainer.S_DRIVERCONTROLS.addDriverRumbleCommand(new RumbleCommand(new SineWave(0.25, 0.3), RumbleType.kRightRumble, 1.0))
    );
  }
}

