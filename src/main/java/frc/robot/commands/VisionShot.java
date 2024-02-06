package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.game.VisionShotLibrary;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.LED;

public class VisionShot extends Command{

    private Launcher mLauncher;
    private Angler mAngler;
    private LED mLED;
    private DriverControls mDriverControls;
    private Timer mTimer;
    private double mTimeout;
    private VisionShotLibrary mVisionShotLibrary;
    boolean mIsNoteExiting;

    
    public VisionShot(Angler angler, Launcher launcher, DriverControls driverControls, LED led, VisionShotLibrary visionShotLibrary, double timeout){
        mLauncher = launcher;
        mAngler = angler;
        mDriverControls = driverControls;
        mLED = led;
        mTimer = new Timer();
        mVisionShotLibrary = visionShotLibrary;
        mTimeout = timeout;
    }

    @Override
    public void end(boolean interrupted) {
        if(mLED.mStateBeforeShooting == LEDState.TELEOP){
            mLED.setTeleopAnimation();
        } else if(mLED.mStateBeforeShooting == LEDState.WANT_AMP){
            mLED.setWantAmpAnimation();
        } else if(mLED.mStateBeforeShooting == LEDState.WANT_COOP){
            mLED.setWantCoopAnimation();
        }
    }

    @Override
    public void execute() {
        if(mAngler.atPosition() == true && mLauncher.isLauncherAtSpeed()){
            mLauncher.sendNoteToFlywheel();
        }
    }

    @Override
    public void initialize() {
        mTimer.start();
        mLED.mStateBeforeShooting = mLED.mState;
        mLED.setShootingAnimation();
        mAngler.toSpeakerPose();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}


