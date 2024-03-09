// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.fieldmirroring;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class BlueAllianceFieldPoseCollection {
    private List<FlippableBlueAlliancePose> mPoseList;

    public BlueAllianceFieldPoseCollection(List<FlippableBlueAlliancePose> poses){
        mPoseList = poses;
    }

    public Pose2d getNearestPose(Alliance currentAlliance, Pose2d pose){
        FlippableBlueAlliancePose nearest = null;
        for(FlippableBlueAlliancePose proposedPose: mPoseList){
            if(nearest == null){
                nearest = proposedPose;
            }
            else{
                if(pose.getTranslation().getDistance(proposedPose.getTranslation(currentAlliance)) < 
                                pose.getTranslation().getDistance(nearest.getTranslation(currentAlliance))){

                    nearest = proposedPose;
                }
            }
        }
        return nearest.getPose(currentAlliance);
    }
}
