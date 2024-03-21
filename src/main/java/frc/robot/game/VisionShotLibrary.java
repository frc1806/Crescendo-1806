// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.game;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class VisionShotLibrary {
    private InterpolatingDoubleTreeMap mFlywheelSpeedDoubleTreeMap;
    private InterpolatingDoubleTreeMap mLaunchAngleDoubleTreeMap;


    public VisionShotLibrary(){
        mFlywheelSpeedDoubleTreeMap = new InterpolatingDoubleTreeMap();
        mLaunchAngleDoubleTreeMap = new InterpolatingDoubleTreeMap();
    }

    /**
     * Adds the shot to the library
     * @param distance the distance the shot is from
     * @param launchAngle the launch angle of the shot, degrees
     * @param flywheelRPM the flywheel speed of the shot, RPM
     * @return the existing Vision shot library, to make declarative definition in RobotContainer cleaner
     */
    public VisionShotLibrary withShotEntry(double distance, double launchAngle, double flywheelRPM)
    {
        mLaunchAngleDoubleTreeMap.put(distance, launchAngle);
        mFlywheelSpeedDoubleTreeMap.put(distance, flywheelRPM);
        return this;
    }

    /**
     * Returns a shot for the given distance to goal.
     * @param distance
     * @return
     */
    public Shot getShotForDistance(double distance){
        return new Shot("Generated Vision Shot", mLaunchAngleDoubleTreeMap.get(distance), mFlywheelSpeedDoubleTreeMap.get(distance));
    }

    
}
