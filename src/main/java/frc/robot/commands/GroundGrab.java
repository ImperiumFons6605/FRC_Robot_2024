package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class GroundGrab extends Command{
    
    private Indexer m_indexer;
    private Intake m_Intake;
    private LEDs m_leds;

    public GroundGrab(Intake intake, Indexer indexer, LEDs leds){
        m_Intake = intake;
        m_indexer = indexer;
        m_leds = leds;
        addRequirements(intake,indexer,leds);
    }

    @Override
    public void execute() {
        m_Intake.run();
        m_indexer.run(IndexerConstants.kIndexerGrabPot);
        m_leds.INcargo();
    }

    @Override
    public void end(boolean interrupted) {
        m_Intake.stop();
        m_indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}