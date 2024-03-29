package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.util.PolarCoordinate;
import frc.robot.util.fieldmirroring.BlueAllianceFieldPoseCollection;
import frc.robot.util.fieldmirroring.FlippableBlueAlliancePose;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class Vision extends SubsystemBase {
    
    private PhotonCamera mFrontLeftCam, mFrontRightCam, mBackLeftCam, mBackRightCam;
    private PhotonPoseEstimator mFrontLeftEstimator, mFrontRightEstimator, mBackLeftEstimator, mBackRightEstimator;
    private FlippableBlueAlliancePose mBlueSpeakerPose, mBlueSubwooferAmp, mBlueSubwooferCenter, mBlueSubwooferAntiAmp, mBlueDriveThruFeeder, mBluePreDriveThruFeeder;
    private FlippableBlueAlliancePose mBlueAmpPose;
    private FlippableBlueAlliancePose mBlueAmpPoseWithSpace;
    private BlueAllianceFieldPoseCollection mBlueAllianceTrapPoses;
    //private BlueAllianceFieldPoseCollection mBlueAllianceSpeakerPoses;
    private static Translation3d ORIGINTRANSLATION = new Translation3d();
    Optional<EstimatedRobotPose> mFrontLeftUpdate, mFrontRightUpdate, mRearLeftUpdate, mRearRightUpdate;
    VisionShotLibrary mVisionShotLibrary;
    
    private AprilTagFieldLayout mFieldLayout;

    public Vision(VisionShotLibrary visionShotLibrary){
        mVisionShotLibrary = visionShotLibrary;
        mFrontLeftCam = new PhotonCamera("LeftFront");
        mFrontRightCam = new PhotonCamera("RightFront");
        mBackLeftCam = new PhotonCamera("LeftRear");
        mBackRightCam = new PhotonCamera("RightRear");

        mFrontLeftUpdate = Optional.empty();
        mFrontRightUpdate = Optional.empty();
        mRearLeftUpdate = Optional.empty();
        mRearRightUpdate = Optional.empty();

        mFrontLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontLeftCam, Constants.kFrontLeftCamToCenter);
        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.kFrontRightCamToCenter);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.kBackLeftCamToCenter);
        mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackRightCam, Constants.kBackRightCamToCenter);

        mFrontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mBlueSpeakerPose = new FlippableBlueAlliancePose(new Translation2d(0.25, 5.55), Rotation2d.fromDegrees(180));
        mBlueSubwooferCenter = new FlippableBlueAlliancePose(new Pose2d(1.36, 5.52, Rotation2d.fromDegrees(180)));
        mBlueSubwooferAntiAmp = new FlippableBlueAlliancePose(new Pose2d(0.76, 4.41, Rotation2d.fromDegrees(120)));
        mBlueSubwooferAmp= new FlippableBlueAlliancePose(new Pose2d(0.76, 6.66, Rotation2d.fromDegrees(-120)));
        mBluePreDriveThruFeeder = new FlippableBlueAlliancePose(new Pose2d(15.94, 2.27, Rotation2d.fromDegrees(-150.0)));
        mBlueDriveThruFeeder = new FlippableBlueAlliancePose(new Pose2d(14.85, 0.74, Rotation2d.fromDegrees(-150.0)));
        mBlueAmpPose = new FlippableBlueAlliancePose(new Translation2d(1.84, 7.64), Rotation2d.fromDegrees(90.0));
        mBlueAmpPoseWithSpace = new FlippableBlueAlliancePose(new Translation2d(1.7384, Units.inchesToMeters(302.244)), Rotation2d.fromDegrees(-90.0));

        ArrayList<FlippableBlueAlliancePose> blueTrapPoses= new ArrayList<>();
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.36, 4.92), Rotation2d.fromDegrees(-60.0)));
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.41, 3.33), Rotation2d.fromDegrees(60.0)));
        blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(5.78, 4.1), Rotation2d.fromDegrees(180.0)));

        /*ArrayList<FlippableBlueAlliancePose> blueSpeakerPoses = new ArrayList<>();
        blueSpeakerPoses.add(new FlippableBlueAlliancePose(new Translation2d(0.2, 5.55), Rotation2d.fromDegrees(180)));
        blueSpeakerPoses.add(new FlippableBlueAlliancePose(new Translation2d(0.2, 5.55), Rotation2d.fromDegrees(180)));*/
        
        mBlueAllianceTrapPoses = new BlueAllianceFieldPoseCollection(blueTrapPoses);

        mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        mFrontLeftEstimator.setFieldTags(mFieldLayout);
        mFrontRightEstimator.setFieldTags(mFieldLayout);
        mBackLeftEstimator.setFieldTags(mFieldLayout);
        mBackRightEstimator.setFieldTags(mFieldLayout);
    }

    public Rotation2d getYawToSpeaker(){
        Translation2d difference = getRobotVelocityCompensatedSpeakerPose().minus(RobotContainer.S_SWERVE.getPose().getTranslation());
        return Rotation2d.fromRadians(PolarCoordinate.toPolarCoordinate(() -> difference.getX(), () -> difference.getY())[1]);
    }


    public Translation2d getRobotVelocityCompensatedSpeakerPose(){
        return getSpeakerTranslation().plus(new Translation2d(
            -(RobotContainer.S_SWERVE.getFieldOrientedVelocity().vxMetersPerSecond * .3),
             -(RobotContainer.S_SWERVE.getFieldOrientedVelocity().vyMetersPerSecond * .3)));
    }

    public Translation2d getSpeakerTranslation(){
        return mBlueSpeakerPose.getTranslation(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getAmpGoalPose2d(){
        return mBlueAmpPose.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getAmpGoalWithSpacePose2d(){
        return mBlueAmpPoseWithSpace.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getNearestTrapPose(){
        return mBlueAllianceTrapPoses.getNearestPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance(), RobotContainer.S_SWERVE.getPose());
    }

    public Double getDistanceToSpeaker(){
        return RobotContainer.S_SWERVE.getPose().getTranslation().getDistance(getRobotVelocityCompensatedSpeakerPose());
    }

    public Pose2d getAmpSideSubwoofer(){
        return mBlueSubwooferAmp.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getCenterSubwoofer(){
        return mBlueSubwooferCenter.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getAntiAmpSubwoofer(){
        return mBlueSubwooferAntiAmp.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getPreDriveThruFeeder(){
        return mBluePreDriveThruFeeder.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public Pose2d getDriveThruFeeder(){
        return mBlueDriveThruFeeder.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
    }

    public double CalculateShotAngle() {
        return mVisionShotLibrary.getShotForDistance(getDistanceToSpeaker()).getPivotAngle();
    }

    public double CalculateShotSpeed() {
        return mVisionShotLibrary.getShotForDistance(getDistanceToSpeaker()).getLauncherSpeed();
    }

    @Override
    public void periodic() {
        mFrontLeftUpdate = mFrontLeftEstimator.update();
        mFrontRightUpdate = mFrontRightEstimator.update();
        mRearLeftUpdate = mBackLeftEstimator.update();
        mRearRightUpdate = mBackRightEstimator.update();
        SmartDashboard.putData(this);

        try{
            if(mFrontLeftUpdate.isPresent() && updateIsValid(mFrontLeftUpdate.get())){
                RobotContainer.S_SWERVE.addVisionMeasurement(mFrontLeftUpdate.get().estimatedPose.toPose2d(), mFrontLeftUpdate.get().timestampSeconds);
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Front left estimator had no update to get");
        }

        try{
            if(mFrontRightUpdate.isPresent() && updateIsValid(mFrontRightUpdate.get())){
                RobotContainer.S_SWERVE.addVisionMeasurement(mFrontRightUpdate.get().estimatedPose.toPose2d(), mFrontRightUpdate.get().timestampSeconds);
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Front right estimator had no update to get");
        }
        
         try{
            if(mRearLeftUpdate.isPresent() && updateIsValid(mRearLeftUpdate.get())){
                RobotContainer.S_SWERVE.addVisionMeasurement(mRearLeftUpdate.get().estimatedPose.toPose2d(), mRearLeftUpdate.get().timestampSeconds);
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Back left estimator had no update to get");
        }
         try{
            if(mRearRightUpdate.isPresent() && updateIsValid(mRearRightUpdate.get())){
                RobotContainer.S_SWERVE.addVisionMeasurement(mRearRightUpdate.get().estimatedPose.toPose2d(), mRearRightUpdate.get().timestampSeconds);
            }
        }
        catch(NoSuchElementException e){
            //System.out.println("Vision.java: Back right estimator had no update to get");
        }

        SmartDashboard.putNumber("Distance To Speaker",getRobotVelocityCompensatedSpeakerPose().getDistance(RobotContainer.S_SWERVE.getPose().getTranslation()));
    }

    public void resetRobotOdometryFromVision(){
        boolean hasBeenReset = false;
        try{
            if(mFrontLeftUpdate.isPresent()  && updateIsValid(mFrontLeftUpdate.get())){
                hasBeenReset= true;

                RobotContainer.S_SWERVE.resetOdometry(mFrontLeftUpdate.get().estimatedPose.toPose2d());
            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
         try{
            if(mFrontRightUpdate.isPresent()  && updateIsValid(mFrontRightUpdate.get())){
                if(!hasBeenReset){
                    hasBeenReset= true;
                    RobotContainer.S_SWERVE.resetOdometry(mFrontRightUpdate.get().estimatedPose.toPose2d());
                }
                else{
                    RobotContainer.S_SWERVE.addVisionMeasurement(mFrontRightUpdate.get().estimatedPose.toPose2d(), mFrontRightUpdate.get().timestampSeconds);
                }

            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
        try{
            if(mRearLeftUpdate.isPresent() && updateIsValid(mRearLeftUpdate.get())){
                if(!hasBeenReset){
                    hasBeenReset= true;
                    RobotContainer.S_SWERVE.resetOdometry(mRearLeftUpdate.get().estimatedPose.toPose2d());
                }
                else{
                    RobotContainer.S_SWERVE.addVisionMeasurement(mRearLeftUpdate.get().estimatedPose.toPose2d(), mRearLeftUpdate.get().timestampSeconds);
                }
 
            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
         try{
            if(mRearRightUpdate.isPresent() &&updateIsValid(mRearRightUpdate.get())){
                if(!hasBeenReset){
                    hasBeenReset= true;
                    RobotContainer.S_SWERVE.resetOdometry(mRearRightUpdate.get().estimatedPose.toPose2d());
                }
                else{
                    RobotContainer.S_SWERVE.addVisionMeasurement(mRearRightUpdate.get().estimatedPose.toPose2d(), mRearRightUpdate.get().timestampSeconds);
                }

            }
        }
        catch(NoSuchElementException e){
            //ignore
        }
    }

    private boolean updateIsValid(EstimatedRobotPose estimatedRobotPose){
        //return updateHasAllNearTargets(estimatedRobotPose);
        return updateHasMultipleTargets(estimatedRobotPose) && updateHasAllNearTargets(estimatedRobotPose);
    }

    private boolean updateHasMultipleTargets(EstimatedRobotPose estimatedRobotPose){
        return estimatedRobotPose.targetsUsed.size() > 1.0;
    }

    private boolean updateHasAllNearTargets(EstimatedRobotPose estimatedRobotPose){
        for(PhotonTrackedTarget target: estimatedRobotPose.targetsUsed){
            double distanceToTag = target.getBestCameraToTarget().getTranslation().getDistance(ORIGINTRANSLATION); 
            if(distanceToTag > 5.5){
                return false;
            }
        }
        return true;
        
    }


    @Override
    public void simulationPeriodic() {
    }

}
