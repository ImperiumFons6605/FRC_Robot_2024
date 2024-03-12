package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.superstruct.Indexer;
import frc.robot.subsystems.superstruct.Intake;
import frc.robot.subsystems.superstruct.SuperStructure;
import frc.robot.subsystems.superstruct.SuperStructure.Goal;

public class GroundGrab extends Command{

    private SuperStructure superStructure;

    public GroundGrab(SuperStructure superStructure){
        this.superStructure = superStructure;
    }

    @Override
    public void execute() {
        superStructure.setGoal(Goal.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.setGoal(Goal.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}