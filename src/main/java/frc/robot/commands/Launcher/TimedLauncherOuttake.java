package frc.robot.commands.Launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class TimedLauncherOuttake extends Command {

    private Timer mTimer;
    private Launcher mLauncher;
    private double mSeconds;

    public TimedLauncherOuttake(Launcher launcher, double seconds){
        mTimer = new Timer();
        mLauncher = launcher;
        mSeconds = seconds;
        addRequirements(launcher);
    }

    @Override
    public void end(boolean interrupted) {
        mLauncher.stop();
    }

    @Override
    public void execute() {
        mLauncher.runMotorsForTimedOuttake(-2);
    }

    @Override
    public void initialize() {
        mTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed(mSeconds);
    }
    
}
