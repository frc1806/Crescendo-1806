package frc.robot.commands.Launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.Launcher;

public class SetLauncher extends Command{

    private Launcher mLauncher;
    private double mSpeed;

    public SetLauncher(Launcher launcher, double speed){
        mLauncher = launcher;
        mSpeed = speed;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        mLauncher.setLauncher(mSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
