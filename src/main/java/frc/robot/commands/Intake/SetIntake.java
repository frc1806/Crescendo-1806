package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Reel;

public class SetIntake extends Command{

    private Reel mIntake;
    private DriverControls mDriverControls;
    private double mSpeed;

    public SetIntake(double speed){
        mIntake = RobotContainer.S_INTAKE;
        mSpeed = speed;
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setIntake(0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        mIntake.setIntake(mSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
