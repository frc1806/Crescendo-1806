package frc.robot;

public class Constants {
    // Driver Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kDebugControllerPort = 2;

    public static final double kAcceptableRotationStickError = 1.0/32.0;

    // Swerve
    public static final double kMaxModuleSpeed = 4.0;
    public static final double kDriveBaseRadius = 6.8125;
    // Translation PID
    public static final double kSwerveAutoPIDP = 5.0;
    public static final double kSwerveAutoPIDI = 0.0;
    public static final double kSwerveAutoPIDD = 0.0;

    // Intake
    public static final double kIntakeP = 0.5;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.1;
    public static final double kIntakeV = 0.0;
    public static final double kIntakeS = 0.0;
    public static final double kIntakeA = 0.0;

    // Angler
    public static final double kAnglerP = 0.1;
    public static final double kAnglerI = 0.0;
    public static final double kAnglerD = 0.0;
    public static final double kAcceptableAngleError = 0.5;
}
