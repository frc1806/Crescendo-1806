package frc.robot.util.rumbleutil;

public class SineWave implements RumbleWave {

    double mPeriod;
    double mAmplitude;

    /**
     * Creates a sine wave for use with rumble
     * @param period period of the sine wave in seconds
     * @param amplitude amplitude of the sine wave [0, 1]
     */
    public SineWave(double period, double amplitude){

        mPeriod = period;
        mAmplitude = amplitude;
    }

    @Override
    public double getOutput(double time) {
        return Math.sin( (time % mPeriod)* (2* Math.PI)) * mAmplitude;
    }
    
}
