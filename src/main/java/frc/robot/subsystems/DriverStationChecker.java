// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverStationChecker extends SubsystemBase {

  private Alliance mCurrentAlliance;
  private Boolean mIsRealMatch;
  private Timer mUpdateTimer;
  private static final String S_SIMREALMATCHKEY = "Sim Real Match?";

  /** Creates a new DriverStationChecker. */
  public DriverStationChecker() {
    mUpdateTimer = new Timer();
    
    SmartDashboard.putBoolean(S_SIMREALMATCHKEY, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(mUpdateTimer.advanceIfElapsed(2.0)){
      updateCurrentAlliance();
      updateIsRealMatch();
    }
    SmartDashboard.putData(this);
  }

  public Alliance getCurrentAlliance(){
    return mCurrentAlliance;
  }

  public Boolean isRealMatch(){
    return mIsRealMatch || SmartDashboard.getBoolean(S_SIMREALMATCHKEY, false);
  }

  private void updateCurrentAlliance(){
    mCurrentAlliance = DriverStation.getAlliance().get();
  }

  private void updateIsRealMatch(){
    MatchType currentMatchType = DriverStation.getMatchType();
    mIsRealMatch = (
       currentMatchType == MatchType.Elimination
       || currentMatchType == MatchType.Practice
       || currentMatchType == MatchType.Qualification
       || DriverStation.isFMSAttached());
}
}


