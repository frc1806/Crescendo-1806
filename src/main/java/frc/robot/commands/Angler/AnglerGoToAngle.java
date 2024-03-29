package frc.robot.commands.Angler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AnglerGoToAngle extends Command{
    protected double wantedAngle;


    public AnglerGoToAngle(double wantedAngle){
        this.wantedAngle = wantedAngle;
        addRequirements(RobotContainer.S_ANGLER);
    }

    @Override
    public void execute() {
        RobotContainer.S_ANGLER.setAngle(wantedAngle);
    }

    @Override
    public void initialize() {
        RobotContainer.S_ANGLER.setAngle(wantedAngle);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.S_ANGLER.atPosition();
    }
    
}