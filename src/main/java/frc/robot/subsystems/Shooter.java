package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

    BangBangController m_leftcontroller = new BangBangController(100);
    BangBangController m_rightcontroller = new BangBangController(100);
    CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftID, MotorType.kBrushless);
    CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightID, MotorType.kBrushless);
    RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

    public Shooter(){
        m_leftMotor.setInverted(true);
    }
    /**
     * set Shooter velocity
     * @param velRPM velocity in RPMs
     */
    public void setVel(double velRPM){
        m_leftcontroller.setSetpoint(velRPM);
        m_rightcontroller.setSetpoint(velRPM);
    }

    public void rawRun(){
        //m_leftMotor.set(1);
        m_rightMotor.set(1);
    }

    public void rawStop(){
        //m_leftMotor.set(0);
        //m_rightMotor.set(0);
    }

    public boolean atSetpoint(){
        if(m_leftcontroller.atSetpoint() && m_rightcontroller.atSetpoint()){
            return true;
        } else {
            return false;
        }
    }

    public double getMeasurement(){
        return ((getLeftMeasurement() + getRightMeasurement()) / 2);
    }

    public double getLeftMeasurement(){
        return m_leftEncoder.getVelocity();
    }

    public double getRightMeasurement(){
        return m_rightEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        m_leftcontroller.calculate(getLeftMeasurement());
        m_rightcontroller.calculate(getRightMeasurement());
        SmartDashboard.putNumber("Shooter Vel", getMeasurement());
    }
}
