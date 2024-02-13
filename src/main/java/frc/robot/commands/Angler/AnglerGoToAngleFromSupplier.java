package frc.robot.commands.Angler;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class AnglerGoToAngleFromSupplier extends Command {
    
    private DoubleSupplier mAngleSupplier;

    public AnglerGoToAngleFromSupplier(DoubleSupplier wantedAngleSupplier) {
        this.mAngleSupplier = wantedAngleSupplier;
        addRequirements(RobotContainer.S_ANGLER);
    }

    @Override
    public void execute() {
        RobotContainer.S_ANGLER.setAngle(mAngleSupplier.getAsDouble());
    }

    @Override
    public void initialize() {
        RobotContainer.S_ANGLER.setAngle(mAngleSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.S_ANGLER.atPosition();
    }


}


