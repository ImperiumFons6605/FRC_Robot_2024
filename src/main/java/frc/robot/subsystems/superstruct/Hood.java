package frc.robot.subsystems.superstruct;

import java.sql.Driver;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.TunableNumber;

public class Hood extends ProfiledPIDSubsystem{
    private static final TunableNumber kP = new TunableNumber("Hood/Gains/Kp", HoodConstants.kHoodP);
    private static final TunableNumber kI = new TunableNumber("Hood/Gains/KI", HoodConstants.kHoodI);
    private static final TunableNumber kD = new TunableNumber("Hood/Gains/KD", HoodConstants.kHoodD);
    private static final TunableNumber kS = new TunableNumber("Hood/Gains/KS", HoodConstants.HoodKS);
    private static final TunableNumber kG = new TunableNumber("Hood/Gains/KG", HoodConstants.HoodkG);
    private static final TunableNumber kV = new TunableNumber("Hood/Gains/KV", HoodConstants.HoodKV);
    private static final TunableNumber maxVelocity = new TunableNumber("Hood/maxVelocity", HoodConstants.kMaxVelocityDegPerSec);
    private static final TunableNumber maxAcceleration = new TunableNumber("Hood/maxAcceleration", HoodConstants.kMaxAccelDegPerSecSquared);
    private static final TunableNumber lowerLimitDegrees = new TunableNumber("Hood/LowerLimitDegrees", HoodConstants.kMinAngleDegrees);
    private static final TunableNumber upperLimitDegrees = new TunableNumber("Hood/UpperLimitDegrees", HoodConstants.kMaxAngleDegrees);

    public static final Supplier<TrapezoidProfile.Constraints> profileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.getAsDouble(), maxAcceleration.getAsDouble());

    public enum Goal{
        STOP(() -> 0.0),
        STORE(new TunableNumber("Hood/StoreAngle", 0.0)),
        INTAKE(new TunableNumber("Hood/IntakeAngle", 45.0)),
        //TODO Create aim supplier function
        AIM(() -> 20.0),
        AMP(new TunableNumber("Hood/AmpAngle", 60.0)),
        SUBWOOFER(new TunableNumber("Hood/SubwooferAngle", 80.0)),
        TRAP(new TunableNumber("Hood/TrapAngle", 70.0));

        private final DoubleSupplier position;
        public DoubleSupplier getGoalPosition() {return position;}
        private Goal(DoubleSupplier supplier){this.position = supplier;}
    }

    private boolean isStopped = false;

    private TrapezoidProfile.Constraints currentConstraints = profileConstraints.get();
    private Hood.Goal goal = Hood.Goal.STORE;

    private CANSparkMax m_motor = new CANSparkMax(HoodConstants.kHoodID, MotorType.kBrushless);
    private DutyCycleEncoder m_encoderOffset = new DutyCycleEncoder(1);
    private Encoder m_encoder = new Encoder(2, 3);
    private double initialPosition;

    private ArmFeedforward m_feedForward;
    
    public Hood() {


        super(new ProfiledPIDController(
        kP.getAsDouble(), 
        kI.getAsDouble(), 
        kD.getAsDouble(), 
        new Constraints(
            maxVelocity.getAsDouble(),
            maxAcceleration.getAsDouble())));
        
        m_feedForward = new ArmFeedforward(
        kS.getAsDouble(), 
        kG.getAsDouble(), 
        kV.getAsDouble());

        m_encoderOffset.setDistancePerRotation(HoodConstants.kHoodEncoderPositionFactor);
        m_encoderOffset.setPositionOffset(HoodConstants.kEncoderZeroOffset);
        //m_encoder.setPositionConversionFactor(HoodConstants.kHoodEncoderPositionFactor);
        //m_encoder.setVelocityConversionFactor(HoodConstants.kHoodEncoderVelocityFactor);
        m_encoder.setDistancePerPulse(HoodConstants.kHoodEncoderPositionFactor);

        this.getController().setTolerance(0.5);
        this.enable();

        m_motor.setSmartCurrentLimit(40);
        m_motor.enableVoltageCompensation(12);
        m_motor.burnFlash();

        if(RobotBase.isSimulation()){
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
        }     
    }

    
    @Override
    protected double getMeasurement() {
        return m_encoder.getDistance();
    }
    public double getMeasurementDegrees(){
        return getMeasurement();
    }

    public double getMeasurementRadians(){
        return Units.degreesToRadians(getMeasurement());
    }
    
    /**
     * @return Current velocity returned in Deg/sec
     */
    public double getVelocityDegPerSec(){
        return m_encoder.getDistance();
    }
    @Override
    public void periodic() {

        TunableNumber.ifChanged(
            hashCode(),
            () -> setPID(
                kP.getAsDouble(),
                kI.getAsDouble(),
                kD.getAsDouble()),
            kP, 
            kI, 
            kD);
        
        TunableNumber.ifChanged(
            hashCode(),
            () -> m_feedForward = new ArmFeedforward(
                kS.getAsDouble(), 
                kG.getAsDouble(), 
                kV.getAsDouble()),
            kS, 
            kG, 
            kV);

        TunableNumber.ifChanged(
            hashCode(),
            () -> getController().setConstraints(
                new Constraints(
                    maxVelocity.getAsDouble(), 
                    maxAcceleration.getAsDouble())), 
            maxVelocity,
            maxAcceleration);
        
        if(DriverStation.isDisabled() || goal == Hood.Goal.STOP ){
            if(this.getController().getGoal().position != goal.getGoalPosition().getAsDouble()){
                setGoal(getMeasurement());
                disable();
                isStopped = true;
            }
        } else {
            if(this.getController().getGoal().position != goal.getGoalPosition().getAsDouble()){
                if(isStopped){
                    enable();
                    isStopped = false;
                }
                setGoal(
                    MathUtil.clamp(
                        getGoal().getGoalPosition().getAsDouble(),
                        lowerLimitDegrees.getAsDouble(),
                        upperLimitDegrees.getAsDouble()
                    )
                );
                System.out.println("SettingGoal to Hood");
            }
        }

        SmartDashboard.putBoolean("Hood Enabled", isEnabled());
        SmartDashboard.putNumber("Hood Controller Goal", getController().getGoal().position);
        
    }


    @Override
    protected void useOutput(double output, State setpoint) {  
        SmartDashboard.putNumber("Hood Output used ", output); 
        double ff = m_feedForward.calculate(setpoint.position, setpoint.velocity);
        m_motor.setVoltage(output + ff);

    }
    
    public void setGoal(Hood.Goal goal){
        this.goal = goal;
    } 

    public Hood.Goal getGoal(){
        return this.goal;
    }

    public boolean atGoal(){
        return this.getController().atGoal();
    }

    public void setPID(double p, double i, double d){
        this.getController().setP(p);
        this.getController().setI(i);
        this.getController().setD(d);
    }
    
}
