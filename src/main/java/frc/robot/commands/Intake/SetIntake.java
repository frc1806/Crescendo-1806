package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Reel;

public class SetIntake extends Command{

    private Reel mIntake;
    private DriverControls mDriverControls;
    private double mSpeed;

    public SetIntake(Reel intake, DriverControls driverControls, double speed){
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
    }

    @Override
    public void initialize() {
        mIntake.setIntake(mSpeed);
    }

    @Override
    public boolean isFinished() {
        return !mDriverControls.intake();
    }

}
