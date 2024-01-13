package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntake extends Command{

    private Intake mIntake;
    private double mSpeed;

    public SetIntake(Intake intake, double speed){
        mIntake = intake;
        mSpeed = speed;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        mIntake.setIntake(mSpeed);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
