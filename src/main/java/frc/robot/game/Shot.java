package frc.robot.game;

public class Shot {
    double pivotAngle;
    String name;
    double launcherSpeed;

    public Shot(String name, double pivotAngle, double launcherSpeed) {
        this.pivotAngle = pivotAngle;
        this.name = name;
        this.launcherSpeed = launcherSpeed;
    }

    //home angle needs to be determined
    public static Shot HOME = new Shot("Home", 330.0, 0.0);
    //make more shots here
    public static Shot SUBWOOFER = new Shot("Subwoofer", 360.0, 1.0);



    public double getPivotAngle() {
        return pivotAngle;
    }

    public double getLauncherSpeed(){
        return launcherSpeed;
    }

    public String getName() {
        return name;
    }
}
