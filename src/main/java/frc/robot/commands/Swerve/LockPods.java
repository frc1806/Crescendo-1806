package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class LockPods extends Command {

    private Swerve mSwerve;

    public LockPods(Swerve swerve){
        mSwerve = swerve;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        mSwerve.lock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
