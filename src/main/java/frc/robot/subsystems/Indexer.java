package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    private CANSparkMax m_motor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushless);
    private DigitalInput m_IRSensor = new DigitalInput(0);

    public Indexer(){
        m_motor.setInverted(true);
    }

    public void run(double vel){
        m_motor.set(vel);
    }

    public void stop(){
        m_motor.set(0);
    }

    public boolean isNoteCharged(){
        return m_IRSensor.get();
    }
}
