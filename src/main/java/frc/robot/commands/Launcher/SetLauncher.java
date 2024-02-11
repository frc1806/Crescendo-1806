package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;

public class SetLauncher extends Command{

    private Launcher mLauncher;
    private double mSpeed;

    public SetLauncher(double speed){
        mLauncher = RobotContainer.S_LAUNCHER;
        mSpeed = speed;
        addRequirements(mLauncher);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        mLauncher.stopIndexer(); //idle the indexer while spinning up flywheel... or stopping flywheel.
    }

    @Override
    public void initialize() {
        mLauncher.setLauncher(mSpeed);
    }

    @Override
    public boolean isFinished() {
        return mLauncher.isLauncherAtSpeed();
    }
    
}
