package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.sequence.IntakeSequence;
import frc.robot.commands.sequence.PresetShotLaunchSequence;
import frc.robot.commands.sequence.VisionShotSequence;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.util.PolarCoordinate;
import frc.robot.util.rumbleutil.RumbleCommand;
import frc.robot.util.rumbleutil.SineWave;
import frc.robot.util.rumbleutil.SquareWave;

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

    private Queue<RumbleCommand> driverRumbleQueue;
    private RumbleCommand currentDriverRumble;
    private Queue<RumbleCommand> operatorRumbleQueue;
    private RumbleCommand currentOperatorRumble;

    public DriverControls(){
        driverController = new XboxController(Constants.kDriverControllerPort);
        operatorController = new XboxController(Constants.kOperatorControllerPort);
        debugController = new XboxController(Constants.kDebugControllerPort);

        SmartDashboard.putNumber("Angle", 0);
        SmartDashboard.putNumber("Launcher Power", 0);
        driverRumbleQueue = new ArrayDeque<>();
        operatorRumbleQueue = new ArrayDeque<>();
        currentDriverRumble = null;
        currentOperatorRumble = null;
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

        return flipValueIfRed(-MathUtil.applyDeadband(driverController.getLeftX(), Constants.kLeftXboxJoystickDeadzone));
    }
    
    public double translationX(){
        return flipValueIfRed(-MathUtil.applyDeadband(driverController.getLeftY(), Constants.kLeftXboxJoystickDeadzone));
    }

    public double snapRotation(){
        DoubleSupplier x = () -> flipValueIfRed(-driverController.getRightX());
        DoubleSupplier y = () -> flipValueIfRed(-driverController.getRightY());

        double[] rightJoyPolarCoordinate = PolarCoordinate.toPolarCoordinate(y,x);
    
        double r = MathUtil.applyDeadband(rightJoyPolarCoordinate[0], Constants.kRightXboxJoystickDeadzone);
        double theta = Units.radiansToDegrees(rightJoyPolarCoordinate[1]);

        if(r < 0.8){
            return RobotContainer.S_SWERVE.getSwerveDrive().getOdometryHeading().getDegrees();
        }

        theta /= 45;
        theta = Math.round(theta) * 45;
        return theta;
    }
    
    public boolean wantPreciseRotation(){
        return driverController.getRightTriggerAxis() > 0;
    }

    public double getPreciseRotationAxis(){
        return -driverController.getRightX();
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

    public boolean o_wantExtendBoatHook(){
        return operatorController.getAButton();
    }
    public boolean o_wantRetractBoatHook(){
        return operatorController.getBButton();
    }
    public boolean o_wantStopBoatHook(){
        return operatorController.getYButton();
    }

    // Debug Controls
    public double d_pivotAnglerManual() {
        return debugController.getRightY();
    }

    public boolean d_wantAnglerManual() {
        return d_pivotAnglerManual() != 0;
    }

    public boolean d_wantDashboardShot(){
        return debugController.getXButton();
    }

    public void registerTriggers(Swerve swerve, Reel intake, Angler angler, Vision vision, Launcher launcher, LED led, BoatHook boatHook, VisionShotLibrary shotLibrary){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
        new Trigger(this::resetGyro).onTrue(new ResetGyro(swerve));
        //new Trigger(this::intake).whileTrue(intake.setIntake(1));
        new Trigger(this::intake).onTrue(new IntakeSequence());
        //Operator
        new Trigger(this::o_wantExtendBoatHook).whileTrue(boatHook.extendBoatHook());
        new Trigger(this::o_wantRetractBoatHook).whileTrue(boatHook.retractBoatHook());
        new Trigger(this::o_wantStopBoatHook).whileTrue(boatHook.stopBoatHook());
        new Trigger(this::o_wantVisionShot).whileTrue(new VisionShotSequence(vision.CalculateShotAngle(), vision.CalculateShotSpeed()));
        

        //Debug
        new Trigger(this::d_wantDashboardShot).whileTrue(new PresetShotLaunchSequence(new Shot(
            "Dashboard Shot",
            SmartDashboard.getNumber("Angle", 0),
            SmartDashboard.getNumber("Launcher Power", 0))));


        //RUMBLE TRIGGERS
        new Trigger(() ->(DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() < 20)).onTrue(addDriverRumbleCommand(new RumbleCommand(new SquareWave(0.3,0.3 ,0.5), RumbleType.kBothRumble, 2.0)));

        new Trigger(() ->(DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() < 10)).onTrue(addDriverRumbleCommand(new RumbleCommand(new SquareWave(0.3,0.5 ,1.0), RumbleType.kBothRumble, 2.0)));

        new Trigger(() ->(driverController.getLeftStickButton())).onTrue(addDriverRumbleCommand(new RumbleCommand(new SquareWave(0.3, 0.7, 1.0), RumbleType.kBothRumble, 1.2)));
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TIMER", DriverStation.getMatchTime());
        updateDriverRumble();
        updateOperatorRumble();
    }

    public Command addDriverRumbleCommand(RumbleCommand command){
        //we explicitly don't want to require this subsystem due to composition fun, we have a queue system to deal with conflicts
        return new InstantCommand(() -> {addRumbleCommandToDriverQueue(command);});
    }

    public Command addOperatorRumbleCommand(RumbleCommand command){
        //we explicitly don't want to require this subsystem due to composition fun, we have a queue system to deal with conflicts
        return new InstantCommand(() -> {addRumbleCommandToOperatorQueue(command);});
    }

    private void updateDriverRumble(){
        if(driverRumbleQueue.isEmpty()){
            if(currentDriverRumble != null){
                if(manageRumbleCommand(currentDriverRumble, driverController)){
                    currentDriverRumble = null;
                }
            }
            else{
                driverController.setRumble(RumbleType.kBothRumble, 0.0);
            }
            
        }
        else{
            if(currentDriverRumble == null){
                currentDriverRumble = driverRumbleQueue.poll();
                if(manageRumbleCommand(currentDriverRumble, driverController)){
                    currentDriverRumble = null;
                }
            }
            else{
                if(manageRumbleCommand(currentDriverRumble, driverController)){
                    currentDriverRumble = null;
                }
            }
        }
    }

        private void updateOperatorRumble(){
        if(operatorRumbleQueue.isEmpty()){
            if(currentOperatorRumble != null){
                if(manageRumbleCommand(currentOperatorRumble, operatorController)){
                    currentOperatorRumble = null;
                }
            }
            else{
                operatorController.setRumble(RumbleType.kBothRumble, 0.0);
            }
            
        }
        else{
            if(currentOperatorRumble == null){
                currentOperatorRumble = operatorRumbleQueue.poll();
                if(manageRumbleCommand(currentOperatorRumble, operatorController)){
                    currentOperatorRumble = null;
                }
            }
            else{
                if(manageRumbleCommand(currentOperatorRumble, operatorController)){
                    currentOperatorRumble = null;
                }
            }
        }
    }

    private boolean manageRumbleCommand(RumbleCommand command, XboxController controller){
        if(!command.hasCommandStarted()){
            command.initialize();
        }
        if(command.isFinished()){
            controller.setRumble(command.getRumbleType(), 0.0);
            command.end();
            return true;
        }else{
            controller.setRumble(command.getRumbleType(), command.getRumbleAmount());
            return false;
        }
    }

    public void addRumbleCommandToDriverQueue(RumbleCommand command){
        driverRumbleQueue.add(command);
    }

    public void addRumbleCommandToOperatorQueue(RumbleCommand command){
        operatorRumbleQueue.add(command);
    }

    @Override
    public void simulationPeriodic() {
    }

    private double flipValueIfRed(double value){
        if(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance() == Alliance.Red){
            return value * -1.0;
        }
        return value;
    }



}
