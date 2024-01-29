package frc.robot.game;


public class Shots {
    double pivotAngle;
    String name;

    public Shots(String name, double pivotAngle) {
        this.pivotAngle = pivotAngle;
        this.name = name;
    }

    //home angle needs to be determined
    public static Shots HOME = new Shots("Home", 154.0);
    //make more shots here
    public static Shots SUBWOOFER = new Shots("Subwoofer", 70.0);



    public double getPivotAngle() {
        return pivotAngle;
    }

    public String getName() {
        return name;
    }
}
