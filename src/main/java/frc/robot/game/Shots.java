package frc.robot.game;


public class Shots {
    Double pivotAngle;
    String name;

    public Shots(String name, Double pivotAngle) {
        this.pivotAngle = pivotAngle;
        this.name = name;
    }

    //home angle needs to be determined
    public static Shots Home = new Shots("Home", 15.0);
    //make more shots here


    public Double getPivotAngle() {
        return pivotAngle;
    }

    public String getName() {
        return name;
    }
}
