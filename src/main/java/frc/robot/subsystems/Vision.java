package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    
    private PhotonCamera mFrontLeftCam, mFrontRightCam, mBackLeftCam, mBackRightCam;
    private PhotonPoseEstimator mFrontLeftEstimator, mFrontRightEstimator, mBackLeftEstimator, mBackRightEstimator;
    private SwerveDrivePoseEstimator mSwerveDrivePoseEstimator;
    private double mDistancex, mDistancey, mTotalDistance;
    private Translation2d mBlueSpeakerPose;
    private Translation2d mRedSpeakerPose;
    
    private AprilTagFieldLayout mFieldLayout;

    private static final Swerve SWERVE = RobotContainer.S_SWERVE;

    public Vision(){
        mFrontLeftCam = new PhotonCamera("FrontLeft");
        mFrontRightCam = new PhotonCamera("FrontRight");
        mBackLeftCam = new PhotonCamera("BackLeft");
        mBackRightCam = new PhotonCamera("BackRight");

        mFrontLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontLeftCam, Constants.kFrontLeftCamToCenter);
        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.kFrontRightCamToCenter);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.kBackLeftCamToCenter);
        mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackRightCam, Constants.kBackRightCamToCenter);

        mFrontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mBlueSpeakerPose = new Translation2d(0.02, 5.61);
        mRedSpeakerPose = new Translation2d(16.53, 5.61);


        mSwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            SWERVE.getKinematics(),
            SWERVE.getHeading(),
            Constants.kSwerveModulePositions,
            new Pose2d(),
            Constants.kStateStds,
            Constants.kVisionStds);

        mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public Double toSpeakerPose(){
        if(Angler.isBlueTeam() == true){
        mDistancex =  RobotContainer.S_VISION.getEstimatedPose().getTranslation().getX() - mBlueSpeakerPose.getX();
        mDistancey =  RobotContainer.S_VISION.getEstimatedPose().getTranslation().getY() - mBlueSpeakerPose.getY();
        mTotalDistance = Math.sqrt(Math.pow(mDistancex, 2) + Math.pow(mDistancey, 2));
        }
        else
        {
        mDistancex =  RobotContainer.S_VISION.getEstimatedPose().getTranslation().getX() - mRedSpeakerPose.getX();
        mDistancey =  RobotContainer.S_VISION.getEstimatedPose().getTranslation().getY() - mRedSpeakerPose.getY();
        mTotalDistance = Math.sqrt(Math.pow(mDistancex, 2) + Math.pow(mDistancey, 2));
        }
        return mTotalDistance;
    }

    public Pose2d getEstimatedPose(){
        return mSwerveDrivePoseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Vision Angle", getEstimatedPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision X", getEstimatedPose().getTranslation().getX());
        SmartDashboard.putNumber("Vision Y", getEstimatedPose().getTranslation().getY());

        if(mFrontLeftEstimator.update().isPresent()){
            mSwerveDrivePoseEstimator.addVisionMeasurement(mFrontLeftEstimator.update().get().estimatedPose.toPose2d(), mFrontLeftEstimator.update().get().timestampSeconds);
        }
        if(mFrontRightEstimator.update().isPresent()){
            mSwerveDrivePoseEstimator.addVisionMeasurement(mFrontRightEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds);
        }
        if(mBackLeftEstimator.update().isPresent()){
            mSwerveDrivePoseEstimator.addVisionMeasurement(mBackLeftEstimator.update().get().estimatedPose.toPose2d(), mBackLeftEstimator.update().get().timestampSeconds);
        }
        if(mBackRightEstimator.update().isPresent()){
            mSwerveDrivePoseEstimator.addVisionMeasurement(mBackRightEstimator.update().get().estimatedPose.toPose2d(), mBackRightEstimator.update().get().timestampSeconds);
        }

    }

    @Override
    public void simulationPeriodic() {
    }

}
