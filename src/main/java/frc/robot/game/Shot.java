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
    public static Shot HOME = new Shot("Home", 247.5, 0.0);
    //make more shots here
    public static Shot SUBWOOFER = new Shot("Subwoofer", 335.0, 5000.0);
    public static Shot BACKWARDS_SUBWOOFER = new Shot("Backwards Subwoofer", 30, 5000);
    public static Shot AMPLIFIER = new Shot("Amplifier", 331.0, 800);
    public static Shot FANCY_AMP_PT_1 = new Shot("Fancy Amp Pt 1", 340.0, 877);
    public static Shot FANCY_AMP_PT_2 = new Shot("Fancy Amp Pt 2", 300.0, 877);
    public static Shot BACKWARDS_AMPLIFIER = new Shot("Backwards Amplifier", 386, 640);
    //old backwards amp 640
    public static Shot CLOSE = new Shot("Close", 350.0, 4000.0);
    public static Shot YEET = new Shot ("Yeet", 305, 4000.0);
    public static Shot TestShot = new Shot("Test", 320.5, 5000);
    public static Shot PassShot = new Shot("Pass", 295, 5500);


    public Shot getHigherShot(){
        return new Shot(name + " Higher", pivotAngle + 2.0, launcherSpeed);
    }

    public Shot getLowerShot(){
        return new Shot(name + " Lower", pivotAngle -2.0, launcherSpeed);
    }

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
