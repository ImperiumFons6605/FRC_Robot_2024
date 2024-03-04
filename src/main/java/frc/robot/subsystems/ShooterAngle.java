package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterAngle extends PIDSubsystem{

    private CANSparkMax m_motor = new CANSparkMax(ShooterConstants.kAngleID, MotorType.kBrushless);
    private AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();

    private ArmFeedforward m_feedForward = new ArmFeedforward(
        ShooterConstants.angleKS, 
        ShooterConstants.anglekG, 
        ShooterConstants.ShooterKV);
    
    public ShooterAngle() {

        super(new PIDController(
        ShooterConstants.kAngleP, 
        ShooterConstants.kAngleI, 
        ShooterConstants.kAngleD),
         0);

         m_encoder.setInverted(true);
        m_encoder.setPositionConversionFactor(ShooterConstants.kAngleEncoderPositionFactor);
    }

    
    @Override
    protected double getMeasurement() {
        return m_encoder.getPosition();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ffOutput = m_feedForward.calculate(getMeasurement(), setpoint);

        m_motor.setVoltage(ffOutput + output);
    }
    
}
