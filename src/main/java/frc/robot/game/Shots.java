package frc.robot.game;


public class Shots {
    double pivotAngle;
    String name;
    double launcherSpeed;

    public Shots(String name, double pivotAngle, double launcherSpeed) {
        this.pivotAngle = pivotAngle;
        this.name = name;
        this.launcherSpeed = launcherSpeed;
    }

    //home angle needs to be determined
    public static Shots HOME = new Shots("Home", 154.0, 0.0);
    //make more shots here
    public static Shots SUBWOOFER = new Shots("Subwoofer", 70.0, 1.0);



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
