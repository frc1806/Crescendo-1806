package frc.robot.game;


public class Shots {
    Double pivotAngle;
    String name;

    public Shots(String name, Double pivotAngle) {
        this.pivotAngle = pivotAngle;
        this.name = name;
    }

    public static Shots ExampleShot = new Shots("Example Shot", 45.0);
    //make more shots here

    public Double getPivotAngle() {
        return pivotAngle;
    }

    public String getName() {
        return name;
    }
}
