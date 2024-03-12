package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.superstruct.Indexer;
import frc.robot.subsystems.superstruct.SuperStructure;
import frc.robot.subsystems.superstruct.SuperStructure.Goal;

public class ShootManual extends Command{
    private Shooter m_shooter;
    private Indexer m_indexer;

    private SuperStructure superStructure;
    private Shooter shooter;

    public ShootManual(SuperStructure superStructure, Shooter shooter){
        this.superStructure = superStructure;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        superStructure.setGoal(Goal.AIM);
        shooter.setGoal(Shooter.Goal.SHOOT);
    }


    @Override
    public void execute() {
        if(shooter.atGoal() && superStructure.atGoal()){
            superStructure.setGoal(Goal.SHOOT);
        }
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
