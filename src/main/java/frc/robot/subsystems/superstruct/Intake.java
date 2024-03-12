package frc.robot.subsystems.superstruct;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.utils.TunableNumber;

public class Intake{

    public enum Goal{
        IDLING(() -> 0.0),
        INTAKING(new TunableNumber("Intake/IntakingVoltage", 8.0)),
        EJECTING(new TunableNumber("Intake/EjectingVoltage", -8.0));

        private final DoubleSupplier position;
        public DoubleSupplier getGoalVoltage() {return position;}
        private Goal(DoubleSupplier supplier){this.position = supplier;}
    }
    private CANSparkMax m_motor = new CANSparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);

    private Intake.Goal goal = Intake.Goal.IDLING;
  
    public Intake(){
        m_motor.setInverted(true);
        m_motor.enableVoltageCompensation(12);
        m_motor.setSmartCurrentLimit(40);
        m_motor.burnFlash();

        if(RobotBase.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
        }
    }

    public Intake.Goal getGoal(){
        return this.goal;
    }

    public void setGoal(Intake.Goal goal){
        this.goal = goal;
    }

    /**
     * @return Current velocity returned in RPM
     */
    public double getVelocityRPM(){
        return m_motor.getEncoder().getVelocity();
    }

    public void periodic(){
        m_motor.setVoltage(getGoal().getGoalVoltage().getAsDouble());
        SmartDashboard.putString("Intake Goal", getGoal().name());
    }
}
