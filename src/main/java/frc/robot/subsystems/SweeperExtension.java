// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SweeperExtension extends SubsystemBase {


  private CANSparkMax mSweeperExtend;
  private RelativeEncoder mSweeperExtensionEncoder;
  private SparkPIDController mSweeperExtensionPID;

  private double mWantedExtension;
  /** Creates a new Sweeper. */
  public SweeperExtension() {


    mSweeperExtend = new CANSparkMax(RobotMap.kSweeperExtendId, MotorType.kBrushless);
    mSweeperExtend.setSmartCurrentLimit(80);
    mSweeperExtensionPID = mSweeperExtend.getPIDController();
    mSweeperExtensionEncoder = mSweeperExtend.getEncoder();
    mSweeperExtensionEncoder.setPositionConversionFactor(Constants.kSweeperExtendConversionFactor);
    mSweeperExtensionPID.setP(Constants.kSweeperExtendkP);
    mSweeperExtensionPID.setI(Constants.kSweeperExtendkI);
    mSweeperExtensionPID.setD(Constants.kSweeperExtendSoftkD);
    mWantedExtension = 0; 
    mSweeperExtensionPID.setReference(mWantedExtension, ControlType.kPosition);

  }

  public void setWantedExtensionInches(double inches){
    mWantedExtension = inches;
    mSweeperExtensionPID.setReference(mWantedExtension, ControlType.kPosition);
  }

  public boolean isAtPosition(){
    return Math.abs(mSweeperExtensionEncoder.getPosition() - mWantedExtension) < Constants.kSweeperAceeptableDistanceDelta; 
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Sweeper Wanted Extension", mWantedExtension);
    //SmartDashboard.putNumber("Sweeper Current Extension", mSweeperExtensionEncoder.getPosition()); 
  }
}
