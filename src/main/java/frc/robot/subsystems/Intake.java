package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase{
    
    private CANSparkMax mIntakeMotor;
    
    public Intake(){
        mIntakeMotor = new CANSparkMax(RobotMap.kIntakeMotorPort, MotorType.kBrushless);

        setIntakeMode(IdleMode.kBrake);
    }

    public void setIntake(double speed){

        if(speed != 0.0){
            setIntakeMode(IdleMode.kCoast);
        }
        else {
            setIntakeMode(IdleMode.kBrake);
        }

        mIntakeMotor.set(speed);
    }

    public void setIntakeMode(IdleMode mode){
        mIntakeMotor.setIdleMode(mode);
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
