package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.VisionShot;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.util.PolarCoordinate;

public class DriverControls extends SubsystemBase{
    
    /*
     * My unusual naming conventions documentation:
     * methodName means the method is tied to the driver controller
     * o_MethodName means the method is tied to the operator controller
     * d_MethodName means the method is tied to the debug controller
    */

    private XboxController driverController;
    private XboxController operatorController;
    private XboxController debugController;
    public double commandedAngle;

    public DriverControls(){
        driverController = new XboxController(Constants.kDriverControllerPort);
        operatorController = new XboxController(Constants.kOperatorControllerPort);
        debugController = new XboxController(Constants.kDebugControllerPort);
    }

    public XboxController getDriverController(){
        return driverController;
    }

    public XboxController getOperatorController(){
        return operatorController;
    }

    public XboxController getDebugController(){
        return debugController;
    }

    // DRIVER CONTROLS

    public double translationY(){
        return -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kLeftXboxJoystickDeadzone);
    }
    
    public double translationX(){
        return -MathUtil.applyDeadband(driverController.getLeftY(), Constants.kLeftXboxJoystickDeadzone);
    }

    public void snapRotation(){
        DoubleSupplier x = () -> -driverController.getRightX();
        DoubleSupplier y = () -> -driverController.getRightY();

        double[] rightJoyPolarCoordinate = PolarCoordinate.toPolarCoordinate(y,x);
    
        double r = MathUtil.applyDeadband(rightJoyPolarCoordinate[0], Constants.kRightXboxJoystickDeadzone);
        double theta = Units.radiansToDegrees(rightJoyPolarCoordinate[1]);

        if(r < 0.8){
            commandedAngle = RobotContainer.S_SWERVE.getSwerveDrive().getOdometryHeading().getDegrees();
        }

        theta /= 45;
        theta = Math.round(theta) * 45;
        commandedAngle = theta;
    }
    
    public boolean wantPreciseRotation(){
        return driverController.getRightTriggerAxis() > 0;
    }

    public boolean intake(){
        return driverController.getRightBumper();
    }

    public boolean lockPods(){
        return driverController.getXButton();
    }

    public boolean resetGyro(){
        return driverController.getYButton();
    }

    public boolean wantVisionAlign(){
        return driverController.getAButton();
    }

    public void setRumble(double speed){
        driverController.setRumble(RumbleType.kBothRumble, speed);
        operatorController.setRumble(RumbleType.kBothRumble, speed);
    }

    // OPERATOR CONTROLS
    public boolean o_wantVisionShot(){
        return operatorController.getXButton();
    }

    public boolean o_wantManualAnglerRotate(){
        return operatorController.getRightTriggerAxis() > 0;
    }

    // Debug Controls
    public double d_pivotAnglerManual() {
        return debugController.getRightY();
    }

    public boolean d_wantAnglerManual() {
        return d_pivotAnglerManual() != 0;
    }

    public void registerTriggers(Swerve swerve, Reel intake, Angler angler, Vision vision, Launcher launcher, LED led){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
        new Trigger(this::resetGyro).onTrue(new ResetGyro(swerve));
        new Trigger(this::intake).whileTrue(new SetIntake(intake, this, 1));
        //Operator
        new Trigger(this::o_wantVisionShot).whileTrue(new VisionShot(angler, launcher, this, led));
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TIMER", DriverStation.getMatchTime());

        if(DriverStation.getMatchTime() < 20){
            setRumble(1);
        }
        else if(DriverStation.getMatchTime() < 10){
            setRumble(1);
        }
    }

    @Override
    public void simulationPeriodic() {
    }



}
