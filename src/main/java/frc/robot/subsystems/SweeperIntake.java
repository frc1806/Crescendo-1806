// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class SweeperIntake extends SubsystemBase {

  private CANSparkMax mSweeperIntake;
  private double mWantedIntakeVolts;
  /** Creates a new SweeperIntake. */
  public SweeperIntake() {
    mSweeperIntake = new CANSparkMax(RobotMap.kSweeperIntakeId, MotorType.kBrushless);
    mSweeperIntake.setSmartCurrentLimit(80);
    mSweeperIntake.setVoltage(0.0);
  }

 public void intakeSweep(){
    mWantedIntakeVolts = Constants.kSweeperIntakeSweepVolts;
  }

  public void intakeAmp(){
    mWantedIntakeVolts = Constants.kSweeperIntakeAmpIntakeVolts;
  }
  
  public void outtakeAmp(){
    mWantedIntakeVolts = Constants.kSweeperIntakeAmpOuttakeVolts;
  }

  public void stopIntake(){
    mWantedIntakeVolts = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mSweeperIntake.setVoltage(mWantedIntakeVolts);
  }
}
