package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Reel extends SubsystemBase{
    
    private CANSparkMax mIntakeMotorNeo;
    private TalonFX mIntakeMotorKracken;
    
    public Reel(){
        if(Constants.kIntakeMotorType == Constants.MotorType.kNEO)
        {
            mIntakeMotorNeo = new CANSparkMax(RobotMap.kIntakeMotorId, MotorType.kBrushless);
            mIntakeMotorNeo.setInverted(true);
            mIntakeMotorNeo.setSmartCurrentLimit(50);
            mIntakeMotorNeo.setOpenLoopRampRate(0.05);
        }
        else if (Constants.kIntakeMotorType == Constants.MotorType.kKracken)
        {
            mIntakeMotorKracken = new TalonFX(RobotMap.kIntakeMotorId);
            mIntakeMotorKracken.setInverted(true);
            CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(80.0);
            OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.05);
            TalonFXConfiguration config = new TalonFXConfiguration().withCurrentLimits(currentLimitConfig).withOpenLoopRamps(openLoopRampsConfigs);
            mIntakeMotorKracken.getConfigurator().apply(config);
        }

    }

    /*public Command setIntake(double speed){
        return this.run(() -> mIntakeMotor.set(speed));
    }*/

    public void setIntake(double speed){
                if(Constants.kIntakeMotorType == Constants.MotorType.kNEO)
        {
            mIntakeMotorNeo.set(speed);
        }
        else if (Constants.kIntakeMotorType == Constants.MotorType.kKracken)
        {
            mIntakeMotorKracken.set(speed);
        }
        
    }

    public void stop(){
        setIntake(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }

}
