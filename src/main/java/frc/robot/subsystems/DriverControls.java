package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathfindLTV;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.ResetOdometryFromVision;
import frc.robot.commands.Swerve.FieldOrientedVisionAlignSpeaker;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.WrappedPathFollowingAmp;
import frc.robot.commands.Swerve.WrappedPathFollowingNearestTrap;
import frc.robot.commands.sequence.FancyAmpSequence;
import frc.robot.commands.sequence.IntakeSequence;
import frc.robot.commands.sequence.OuttakeSequence;
import frc.robot.commands.sequence.ParallelCleaningGroup;
import frc.robot.commands.sequence.PresetShotLaunchSequence;
import frc.robot.commands.sequence.VisionShotSequence;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.util.PolarCoordinate;
import frc.robot.util.rumbleutil.RumbleCommand;
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

    private double lastTranslationAngle;

    private double lastSnapDegree;

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
        lastSnapDegree = RobotContainer.S_SWERVE.getSwerveDrive().getOdometryHeading().getDegrees();
        lastTranslationAngle = 0;
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
    
    public double getRadDriveThrottle(){
        return 0;
        //return MathUtil.applyDeadband(driverController.getRightTriggerAxis(), Constants.kTriggerXboxDeadzone);
    }
    public Translation2d getRadDriveTranslation(){
        double[] translationCoord = PolarCoordinate.toPolarCoordinate(() ->translationX(), () ->translationY());
        if(translationCoord[0] < 0.5){
            return new Translation2d(getRadDriveThrottle(), Rotation2d.fromRadians(lastTranslationAngle));
        }
        else{
            lastTranslationAngle = translationCoord[1];
            return new Translation2d(getRadDriveThrottle(), Rotation2d.fromRadians(translationCoord[1]));
        }
        
    }

    public double snapRotation(){
        DoubleSupplier x = () -> flipValueIfRed(-driverController.getRightX());
        DoubleSupplier y = () -> flipValueIfRed(-driverController.getRightY());

        double[] rightJoyPolarCoordinate = PolarCoordinate.toPolarCoordinate(y,x);
    
        double r = rightJoyPolarCoordinate[0];
        double theta = Units.radiansToDegrees(rightJoyPolarCoordinate[1]);

        if(r < 0.8){
            return lastSnapDegree;
        }

        theta /= 30;
        theta = Math.round(theta) * 30;
        lastSnapDegree = theta;
        return theta;
    }

    public void setLastSnapDegree(double newLastSnapDegree){
        lastSnapDegree = newLastSnapDegree;
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

    public boolean outtake(){
        return driverController.getLeftBumper();
    }

    public boolean lockPods(){
        return driverController.getXButton();
    }

    public boolean resetGyro(){
        return driverController.getYButton();
    }

    public boolean wantResetRobotOdometryFromVision(){
        return driverController.getPOV() == 0;
    }

    public boolean wantVisionAlign(){
        return driverController.getAButton();
    }

    public boolean wantVisionAlignAmp(){
        return driverController.getBButton();
    }
    
    public boolean wantVisionAlignNearestTrap(){
        return driverController.getStartButton();
    }

    public void setRumble(double speed){
        driverController.setRumble(RumbleType.kBothRumble, speed);
        operatorController.setRumble(RumbleType.kBothRumble, speed);
    }

  
    public boolean o_wantVisionShot(){
        return driverController.getLeftTriggerAxis() > 0;
    }

      // OPERATOR CONTROLS

    public boolean o_wantSubwooferShot(){
        return operatorController.getYButton();
    }

    public boolean o_wantAmpShot(){
        return operatorController.getAButton();
    }

    public boolean o_wantManualAnglerRotate(){
        return operatorController.getRightBumper();
    }

    public boolean o_wantExtendBoatHook(){
        return operatorController.getRightTriggerAxis() > 0;
    }
    public boolean o_wantRetractBoatHook(){
        return operatorController.getLeftTriggerAxis() > 0;
    }
    public boolean o_wantStopBoatHook(){
        return operatorController.getLeftTriggerAxis() == 0 && operatorController.getRightTriggerAxis() == 0;
    }

    public boolean o_wantModifyShotHigher(){
        //TODO: Make this work for other shots
        return operatorController.getPOV() == 0 && o_wantSubwooferShot();
    }

    public boolean o_wantModifyShotLower(){
        return operatorController.getPOV() == 180 && o_wantSubwooferShot();
    }

    // Debug Controls
    public double d_pivotAnglerManual() {
        return debugController.getRightY();
    }

    public boolean d_wantAnglerManual() {
        return false;
        //return d_pivotAnglerManual() != 0;
    }

    public boolean d_wantDashboardShot(){
        return debugController.getXButton();
    }

    public boolean d_wantCloseShot(){
        return debugController.getAButton();
    }

    public boolean d_wantYeet(){
        return debugController.getYButton();
    }

    public boolean d_wantCleaning(){
        return debugController.getStartButton();
    }

    public void registerTriggers(Swerve swerve, Reel intake, Angler angler, Vision vision, Launcher launcher, LED led, BoatHook boatHook, VisionShotLibrary shotLibrary){
        //Driver
        new Trigger(this::lockPods).onTrue(new LockPods(swerve));
        new Trigger(this::resetGyro).onTrue(new ResetGyro(swerve));
        new Trigger(this::intake).whileTrue(new IntakeSequence());
        new Trigger(this::outtake).whileTrue(new OuttakeSequence());
        new Trigger(this::wantVisionAlign).whileTrue(new FieldOrientedVisionAlignSpeaker());
        new Trigger(this::wantVisionAlignAmp).whileTrue(new WrappedPathFollowingAmp());
        new Trigger(this::wantVisionAlignNearestTrap).whileTrue(new WrappedPathFollowingNearestTrap());
        new Trigger(this::wantResetRobotOdometryFromVision).whileTrue(new ResetOdometryFromVision());
        //Operator
        new Trigger(this::o_wantExtendBoatHook).whileTrue(boatHook.extendBoatHook());
        new Trigger(this::o_wantRetractBoatHook).whileTrue(boatHook.retractBoatHook());
        new Trigger(this::o_wantStopBoatHook).whileTrue(boatHook.stopBoatHook());
        new Trigger(this::o_wantVisionShot).whileTrue(new VisionShotSequence(vision.CalculateShotAngle(), vision.CalculateShotSpeed()));

        new Trigger(this::o_wantSubwooferShot).whileTrue(new PresetShotLaunchSequence(Shot.SUBWOOFER));
        //TODO: Higher/Lower currently only work for subwoofer
        new Trigger(this::o_wantModifyShotHigher).whileTrue(new PresetShotLaunchSequence(Shot.SUBWOOFER.getHigherShot()));
        new Trigger(this::o_wantModifyShotLower).whileTrue(new PresetShotLaunchSequence(Shot.SUBWOOFER.getLowerShot()));

        new Trigger(this::o_wantAmpShot).whileTrue(new FancyAmpSequence());
        //Debug
        new Trigger(this::d_wantDashboardShot).whileTrue(new PresetShotLaunchSequence(new Shot(
            "Dashboard Shot",
            SmartDashboard.getNumber("Angle", 300.0),
            SmartDashboard.getNumber("Launcher Power", 3000.0))));

        new Trigger(this::d_wantCloseShot).whileTrue(new PresetShotLaunchSequence(Shot.CLOSE));
        new Trigger(this::d_wantYeet).whileTrue(new PresetShotLaunchSequence(Shot.YEET));
        new Trigger(this::d_wantCleaning).whileTrue(new ParallelCleaningGroup());

        

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
