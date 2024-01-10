package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Swerve.LockPods;

public class DriverControls {
    
    /*
     * My unusual naming conventions documentation:
     * methodName means the method is tied to the driver controller
     * oMethodName means the method is tied to the operator controller
     * dMethodName means the method is tied to the debug controller
    */


    private XboxController driverController;
    private XboxController operatorController;
    private XboxController debugController;

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
        return driverController.getLeftY();
    }

    public double translationX(){
        return driverController.getLeftX();
    }

    public double rotation(){
        double joystickFieldX = driverController.getRightX();
        double joystickFieldY = driverController.getRightY();

        double xyMag = Math.hypot(joystickFieldX, joystickFieldY);
        double angle = Math.acos(joystickFieldX / xyMag) * (joystickFieldY > 0? 1:-1);
        return angle;
    }

    private boolean lockPods(){
        return driverController.getXButton();
    }

    public void registerTriggers(Swerve swerve){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
    }

}
