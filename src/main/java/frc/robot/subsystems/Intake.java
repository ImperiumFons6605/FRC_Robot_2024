package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private CANSparkMax m_motor = new CANSparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);
    

    public Intake(){
        m_motor.setInverted(true);
    }

    public void run(){
        m_motor.set(IntakeConstants.kIntakePot);
    }

    public void stop(){
        m_motor.set(0);
    }
}
