package frc.robot.subsystems.superstruct;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.utils.TunableNumber;

public class Climber extends ProfiledPIDSubsystem {
    
    private CANSparkMax m_leader = new CANSparkMax(ClimberConstants.kClimberMasterCANId, MotorType.kBrushless);
    private CANSparkMax m_slave = new CANSparkMax(ClimberConstants.kClimberSlaveCANId, MotorType.kBrushless);
    private AbsoluteEncoder m_encoder = m_leader.getAbsoluteEncoder();

    public enum State{
        STOP(()->0.0),
        RETRACTING(new TunableNumber("Climber/RetractingPosition",Units.inchesToMeters(0))),
        EXTENDING(new TunableNumber("Climber/ExtendingPosition",Units.inchesToMeters(10)));

        private final DoubleSupplier position;
        public DoubleSupplier getGoalPosition() {return position;}
        private State(DoubleSupplier supplier){this.position = supplier;}
        
    }

    private State currentState = State.RETRACTING;
    private State desiredState = State.RETRACTING;
    private boolean isStopped = false;

    public Climber () {
        super(
            new ProfiledPIDController(
                ClimberConstants.kClimberP,
                ClimberConstants.kClimberI,
                ClimberConstants.kClimberD,
                new TrapezoidProfile.Constraints(
                    ClimberConstants.kMaxVelocityMetersPerSecond,
                    ClimberConstants.kMaxAccelerationMetersPerSecSquared)),
            0);

        m_leader.setSmartCurrentLimit(40);
        m_slave.setSmartCurrentLimit(40);
        m_leader.enableVoltageCompensation(12);
        m_slave.enableVoltageCompensation(12);

        m_slave.burnFlash();
        m_leader.burnFlash();

        m_slave.setInverted(ClimberConstants.kEncoderInverted);
        m_slave.follow(m_leader);


        
        m_encoder.setPositionConversionFactor(ClimberConstants.kEncoderPositionFactor);
        m_encoder.setVelocityConversionFactor(ClimberConstants.kEncoderVelocityFactor);
        m_encoder.setZeroOffset(ClimberConstants.kEncoderZeroOffset);

        this.getController().setTolerance(0.02);
        this.enable();

        if(RobotBase.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(m_leader, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_slave, DCMotor.getNEO(1));
        }      

    }

    @Override
    public void periodic() {
        currentState = desiredState;

        if(this.getController().getGoal().position != currentState.getGoalPosition().getAsDouble()){
            if(isStopped){
                enable();
                isStopped = false;
            }
            setGoal(
                MathUtil.clamp(
                    currentState.getGoalPosition().getAsDouble(), 
                    ClimberConstants.kMinDistanceMeter, 
                    ClimberConstants.kMaxDistanceMeter)); 
        }

        if((getState() == Climber.State.RETRACTING) && (atGoal())){
            isStopped = true;
            disable();
            m_leader.stopMotor();
            m_slave.stopMotor();
        }

        SmartDashboard.putString("Climber Goal", getState().name());
    }


    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_leader.set(output);
    }

    /**
     * @returns measurement Longitud del climber en Metros
     */
    @Override
    protected double getMeasurement() {
        return m_encoder.getPosition();
    }

    /**
     * @return Current velocity returned in m/s
     */
    public double getVelocityMetersPerSec(){
        return m_encoder.getVelocity();
    }

    public void setGoal(Climber.State state){
        if (desiredState == state) return;
        desiredState = state;
    } 
    

    public State getState(){
        return currentState;
    }

    public boolean atGoal(){
        return getController().atGoal();
    }

    public boolean retracted(){
        return getState() == State.RETRACTING && atGoal();
    }

    public boolean extended(){
        return getState() == State.EXTENDING && atGoal();
    }
}
