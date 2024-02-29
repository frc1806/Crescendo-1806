// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.rumbleutil;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/** Add your docs here. */
public class RumbleCommand {

    Timer mTimer;
    RumbleWave mWave;
    double mTimeToRumble;
    boolean mHasStarted;
    RumbleType mRumbleType;

    /**
     * Creates a command to rumble the controller
     * @param wave the wave to use to rumble the controller
     * @param rumbleType the type of rumble {@link RumbleType}
     * @param timeToRumble how long to rumble for
     */
    public RumbleCommand(RumbleWave wave, RumbleType rumbleType, double timeToRumble){
        mWave = wave;
        mTimeToRumble = timeToRumble;
        mTimer = new Timer();
        mHasStarted = false;
        mRumbleType = rumbleType;
    }

    /**
     * returns true if the rumble command has been started
     * @return
     */
    public boolean hasCommandStarted(){
        return mHasStarted;
    }

    /**
     * starts the rumble command
     */
    public void initialize(){
        mTimer.restart();
        mHasStarted = true;
    }

    /**
     * get the output value to send to the controller for rumble
     * @return a double representing the output value
     */
    public double getRumbleAmount(){
        return mWave.getOutput(mTimer.get());
    }

    public RumbleType getRumbleType(){
        return mRumbleType;
    }
    /**
     * 
     * @return true if the time to rumble has elapsed.
     */
    public boolean isFinished(){
        return mTimer.hasElapsed(mTimeToRumble);
    }

    public void end(){
        mHasStarted = false;
        mTimer.stop();
    }
}
