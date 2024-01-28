package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.DriverControls;

public class VisionShot extends Command{

    private Launcher mLauncher;
    private Angler mAngler;
    private DriverControls mDriverControls;

    
    public VisionShot(Angler angler, Launcher launcher, DriverControls driverControls){
        mLauncher = launcher;
        mAngler = angler;
        mDriverControls = driverControls;

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        if(mAngler.atPosition() == true){
            mLauncher.sendNoteToFlywheel();
        }
    }

    @Override
    public void initialize() {
        mAngler.toSpeakerPose();
    }

    @Override
    public boolean isFinished() {
        return !mDriverControls.wantVisionAlign();
    }
    
}


