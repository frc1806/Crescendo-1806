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
    public static Shot HOME = new Shot("Home", 250.0, 0.0);
    //make more shots here
    public static Shot SUBWOOFER = new Shot("Subwoofer", 330.0, 5000.0);
    public static Shot CLOSE = new Shot("Close", 350.0, 4000.0);
    public static Shot YEET = new Shot ("Yeet", 305, 4000.0);



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
