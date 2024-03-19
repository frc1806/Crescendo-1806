// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.fieldmirroring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.PolarCoordinate;

/** Add your docs here. */
public class FlippableBlueAlliancePose extends Pose2d{

    public FlippableBlueAlliancePose(Pose2d pose){
        super(pose.getTranslation(), pose.getRotation());
    }
    
    public FlippableBlueAlliancePose(Translation2d translation2d, Rotation2d fromDegrees) {
        super(translation2d, fromDegrees);
    }

    public FlippableBlueAlliancePose(){
        super();
    }

    public Pose2d getPose(Alliance currAlliance){
        return new Pose2d(getTranslation(currAlliance), getRotation(currAlliance));
    }

    public Translation2d getTranslation(Alliance currentAlliance){
        if(currentAlliance == Alliance.Blue){
            return getTranslation();
        }
        else{
            return new Translation2d(16.54 - this.getX(), this.getY()); //Mirror across midfield
        }
    }

    public Rotation2d getRotation(Alliance currAlliance){
        if(currAlliance == Alliance.Blue){
            return getRotation();
        }
        else{
            double x = getRotation().getCos();
            double y = getRotation().getSin();
            return Rotation2d.fromRadians(PolarCoordinate.toPolarCoordinate(() ->-x,() -> y)[1]);
        }

    }
}
