// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.rumbleutil;

/** Add your docs here. */
public class SquareWave implements RumbleWave{

    double mPeriod;
    double mDutyCycle;
    double mAmplitude;
    /**
     * Creates a square wave for use in rumble
     * @param period the period of the square wave in seconds
     * @param dutyCycle the duty cycle of the square wave in percent [0, 1]
     * @param amplitude the amplitude of the square wave [0, 1]
     */
    public SquareWave(double period, double dutyCycle, double amplitude){
        mPeriod = period;
        mDutyCycle = dutyCycle;
        mAmplitude = amplitude;
    }
    @Override
    public double getOutput(double time) {
        
       return ((time % mPeriod)/mPeriod) < mDutyCycle? mAmplitude:0.0;
    }
}
