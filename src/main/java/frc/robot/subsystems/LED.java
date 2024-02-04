package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class LED extends SubsystemBase{

    private CANdle mLED;
    public enum LEDState{
        IDLE,
        TELEOP,
        WANT_AMP,
        WANT_COOP,
        SHOOTING,
        ESTOP,
        TEST_MODE
    }
    public LEDState mState;
    public LEDState mStateBeforeShooting;

    public LED(){
        mLED = new CANdle(RobotMap.kCANDleId);
        setIdleAnimation();
    }

    public void setIdleAnimation(){
        mLED.animate(new TwinkleOffAnimation(0,255,0,0, Constants.kTwinkleOffAnimationSpeed, Constants.kNumLED, TwinkleOffAnimation.TwinkleOffPercent.Percent18));
        mState = LEDState.IDLE;
        mStateBeforeShooting = mState;
    }

    public void setTeleopAnimation(){
        int[] colors = {255, 0, 0};

        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            colors[0] = 0;
            colors[2] = 255;
        }

        mLED.animate(new ColorFlowAnimation(colors[0], colors[1], colors[2], 0, Constants.kColorFlowAnimationSpeed, Constants.kNumLED, Direction.Forward));
        mState = LEDState.TELEOP;
    }

    public void setWantAmpAnimation(){
        mLED.animate(new StrobeAnimation(0, 255, 255, 0, Constants.kStrobeAnimationSpeed, Constants.kNumLED));
        mState = LEDState.WANT_AMP;
    }

    public void setWantCoopAnimation(){
        mLED.animate(new StrobeAnimation(255, 255, 0, 0, Constants.kStrobeAnimationSpeed, Constants.kNumLED));
        mState = LEDState.WANT_COOP;
    }

    public void setShootingAnimation(){
        mLED.animate(new StrobeAnimation(0, 0, 0, 255, Constants.kStrobeAnimationSpeed, Constants.kNumLED));
        mState = LEDState.SHOOTING;
    }

    public void setEstopAnimation(){
        mLED.animate(new LarsonAnimation(255, 0, 0));
        mState = LEDState.ESTOP;
    }

    public void setTestModeAnimation(){
        mLED.animate(new RgbFadeAnimation(1, Constants.kRGBFadeAnimationSpeed, Constants.kNumLED));
        mState = LEDState.TEST_MODE;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("LED State", mState.toString());

        if(RobotState.isDisabled()){
            if(mState != LEDState.IDLE){
                setIdleAnimation();
            }
        } else if(RobotState.isTeleop()){
            if(mState != LEDState.TELEOP){
                setTeleopAnimation();
            }
        } else if(RobotState.isEStopped()){
            if(mState != LEDState.ESTOP){
                setEstopAnimation();
            }
        }
    }

    @Override
    public void simulationPeriodic() {
    }
    
}
