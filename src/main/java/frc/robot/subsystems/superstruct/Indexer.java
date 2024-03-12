package frc.robot.subsystems.superstruct;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;
import frc.utils.TunableNumber;

public class Indexer{

    public enum Goal{
        IDLING(() -> 0.0),
        INTAKING(new TunableNumber("Indexer/IntakingVoltage", 6.0)),
        FEED_SHOOTER(new TunableNumber("Indexer/ShootingVoltage", 12.0)),
        EJECTING(new TunableNumber("Indexer/EjectingVoltage", -8.0));

        private final DoubleSupplier position;
        public DoubleSupplier getGoalVoltage() {return position;}
        private Goal(DoubleSupplier supplier){this.position = supplier;}
    }

    private CANSparkMax m_motor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushless);
    private Indexer.Goal goal = Indexer.Goal.IDLING;

    public Indexer(){
        m_motor.setInverted(true);
        m_motor.enableVoltageCompensation(12);
        m_motor.setSmartCurrentLimit(30);
        m_motor.burnFlash();

        if(RobotBase.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
        }
    }

    public Indexer.Goal getGoal(){
        return this.goal;
    }

    public void setGoal(Indexer.Goal goal){
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
        SmartDashboard.putString("Indexer Goal", getGoal().name());
    }
}
