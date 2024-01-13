package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class ResetGyro extends Command {

    private Swerve mSwerve;

    public ResetGyro(Swerve swerve){
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
        mSwerve.zeroGryo();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
