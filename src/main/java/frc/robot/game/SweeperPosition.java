// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.game;

/** Add your docs here. */
public class SweeperPosition {
    double mAngle;
    double mDistance;

    public static SweeperPosition AMP_SCORE = new SweeperPosition(Shot.HOME.getPivotAngle(), 14.0);
    public static SweeperPosition INTAKE = new SweeperPosition(330, 8.0);

    public SweeperPosition(double angle, double distance){
        mAngle= angle;
        mDistance = distance;
    }

    public double getAngle(){
        return mAngle;
    }

    public double getDistance(){
        return mDistance;
    }
}
