package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Intake;

public class SetIntake extends Command{

    private Intake mIntake;
    private DriverControls mDriverControls;
    private double mSpeed;

    public SetIntake(Intake intake, DriverControls driverControls, double speed){
        mIntake = intake;
        mDriverControls = driverControls;
        mSpeed = speed;
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setIntake(0);
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
        return !mDriverControls.intake();
    }

}
