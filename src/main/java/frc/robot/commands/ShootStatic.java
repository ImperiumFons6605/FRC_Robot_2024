package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootStatic extends Command{
    
    private Shooter m_shooter;
    private Indexer m_indexer;

    public ShootStatic(Shooter shooter){
        m_shooter = shooter;
        //m_indexer = indexer;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.rawRun();
        //m_shooter.setVel(3000);
        //m_indexer.run(IndexerConstants.kIndexerShootPot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.rawStop();
        //m_indexer.stop();
    }

}
