package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

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

    public boolean rotateCenter(){
        return driverController.getRightY() > 0;
    }

    public boolean rotateAbout(){
        return driverController.getRightY() < 0;
    }

    public boolean rotateLeft(){
        return driverController.getRightX() < 0;
    }

    public boolean rotateRight(){
        return driverController.getRightX() > 0;
    }

    public void registerTriggers(Swerve swerve){
        //Driver
    }

}
