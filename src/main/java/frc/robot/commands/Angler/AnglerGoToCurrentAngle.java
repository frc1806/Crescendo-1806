package frc.robot.commands.Angler;

import frc.robot.RobotContainer;
import frc.robot.game.Shot;

public class AnglerGoToCurrentAngle extends AnglerGoToAngle {
  /** Creates a new ArmGoToCurrentAngle. */
  public AnglerGoToCurrentAngle() {
    super(Shot.HOME.getPivotAngle()); //this will be later ignored.
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wantedAngle = RobotContainer.S_ANGLER.getAngle(); //Set wanted angle to current angle
    RobotContainer.S_ANGLER.setAngle(wantedAngle);
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}