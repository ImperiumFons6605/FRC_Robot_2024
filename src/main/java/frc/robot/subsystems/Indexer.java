package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    private CANSparkMax m_motor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushless);

    public Indexer(){
        m_motor.setInverted(true);
    }

    public void run(){
        m_motor.set(IndexerConstants.kIndexerPot);
    }

    public void stop(){
        m_motor.set(0);
    }
}
