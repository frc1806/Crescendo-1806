package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase{
    
    private CANSparkMax mIntakeMotor;
    private PIDController mIntakePID;
    private SimpleMotorFeedforward mIntakeFeedForward;
    
    public Intake(){
        mIntakeMotor = new CANSparkMax(RobotMap.kIntakeMotorPort, MotorType.kBrushless);
        mIntakePID = new PIDController(Constants.kIntakeP, Constants.kIntakeI, Constants.kIntakeD);
        mIntakeFeedForward = new SimpleMotorFeedforward(Constants.kIntakeS, Constants.kIntakeV, Constants.kIntakeA);

        setIntakeMode(IdleMode.kBrake);
    }

    public void setIntake(double speed){
        mIntakeMotor.set(mIntakeFeedForward.calculate(mIntakePID.calculate(speed)));
    }

    public void setIntakeMode(IdleMode mode){
        mIntakeMotor.setIdleMode(mode);
    }

    public void stop(){
        setIntake(0.0);
        setIntakeMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    @Override
    public void simulationPeriodic() {
    }

}
