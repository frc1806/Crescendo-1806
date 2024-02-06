package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Launcher.SetLauncher;
import frc.robot.game.Shot;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.LED;

public class PresetShot extends Command{

    private Launcher mLauncher;
    private Angler mAngler;
    private LED mLED;
    private DriverControls mDriverControls;
    private Shot mShot;
    
    public PresetShot(Angler angler, Launcher launcher, DriverControls driverControls, LED led, Shot shot){
        mLauncher = launcher;
        mAngler = angler;
        mDriverControls = driverControls;
        mLED = led;
        mShot = shot;
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
        if(mAngler.atPosition() == true){
            //CommandScheduler.getInstance().schedule(new SetLauncher(mLauncher, mShot.getLauncherSpeed()));
        }
    }

    @Override
    public void initialize() {
        mLED.mStateBeforeShooting = mLED.mState;
        mLED.setShootingAnimation();
        mAngler.setAngle(mShot.getPivotAngle());
    }

    @Override
    public boolean isFinished() {
        return !mDriverControls.wantVisionAlign();
    }
    
}


