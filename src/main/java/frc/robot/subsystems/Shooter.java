package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LinearProfile;
import frc.utils.TunableNumber;

public class Shooter extends SubsystemBase{

    private static final TunableNumber kP = new TunableNumber("Shooter/Gains/Kp", ShooterConstants.kShooterP);
    private static final TunableNumber kI = new TunableNumber("Shooter/Gains/KI", ShooterConstants.kShooterI);
    private static final TunableNumber kD = new TunableNumber("Shooter/Gains/KD", ShooterConstants.kShooterD);
    private static final TunableNumber kS = new TunableNumber("Shooter/Gains/KS", ShooterConstants.ShooterKS);
    private static final TunableNumber kV = new TunableNumber("Shooter/Gains/KV", ShooterConstants.ShooterKV);
    private static final TunableNumber kA = new TunableNumber("Shooter/Gains/KA", ShooterConstants.ShooterKA);
    private static final TunableNumber shootingLeftRpm = new TunableNumber("Shooter/ShootingLeftRpm", ShooterConstants.ShooterLeftRpm);
    private static final TunableNumber shootingRightRpm = new TunableNumber("Shooter/ShootingRightRpm", ShooterConstants.ShooterRightRpm);
    private static final TunableNumber ejectRpm = new TunableNumber("Shooter/EjectRpm", ShooterConstants.EjectingRpm);

    private static final TunableNumber prepareShootMultiplier = new TunableNumber("Shooter/prepareShootMultiplier", ShooterConstants.prepareShootMultiplier);
    private static final TunableNumber maxAcceleration = new TunableNumber("Shooter/MaxAccelerationRpmPerSec", ShooterConstants.maxAccelerationRpmPerSec);

    private final LinearProfile leftProfile;
    private final LinearProfile rightProfile;

    SimpleMotorFeedforward m_ffController = new SimpleMotorFeedforward(
        kS.getAsDouble(), 
        kV.getAsDouble(),
        kA.getAsDouble());


    PIDController m_leftPidController  = new PIDController(
        kP.getAsDouble(), 
        kI.getAsDouble(), 
        kD.getAsDouble());
    PIDController m_rightPidController  = new PIDController(
        kP.getAsDouble(), 
        kI.getAsDouble(), 
        kD.getAsDouble());

    public void setPID(double p, double i, double d){
        m_leftPidController.setP(p);
        m_leftPidController.setI(i);
        m_leftPidController.setD(d);
        m_rightPidController.setP(p);
        m_rightPidController.setI(i);
        m_rightPidController.setD(d);
    }

    private boolean wasClosedLoop = false;
    private boolean closedLoop = false;
    private BooleanSupplier prepareShootSupplier = () -> false;

    public void setPrepareShootSupplier(BooleanSupplier prepareShootSupplier){
        this.prepareShootSupplier = prepareShootSupplier;
    }


    CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftID, MotorType.kBrushless);
    CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightID, MotorType.kBrushless);
    RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();


    public enum Goal {
        IDLE(() -> 0.0, () -> 0.0),
        SHOOT(shootingLeftRpm, shootingRightRpm),
        EJECT(ejectRpm, ejectRpm);

        private final DoubleSupplier leftGoal;
        private final DoubleSupplier rightGoal;
        public DoubleSupplier getLeftGoalRpm() {return leftGoal;}
        public DoubleSupplier getRightGoalRpm() {return rightGoal;}
        private Goal(DoubleSupplier left, DoubleSupplier right){this.leftGoal = left; this.rightGoal = right;}
    }

    public enum IdleMode{
        TELEOP,
        AUTO
    }

    private Shooter.Goal goal = Shooter.Goal.IDLE;
    public Shooter.Goal getGoal(){
        return this.goal;
    }

    public Shooter(){
        leftProfile = new LinearProfile(maxAcceleration.getAsDouble(), 0.02);
        rightProfile = new LinearProfile(maxAcceleration.getAsDouble(), 0.02);

        m_leftMotor.setInverted(true);
        setDefaultCommand(runOnce(()-> setGoal(Shooter.Goal.IDLE)).withName("Shooter Idle"));

        m_rightMotor.setSmartCurrentLimit(50);
        m_leftMotor.setSmartCurrentLimit(50);
        m_rightMotor.enableVoltageCompensation(12);
        m_leftMotor.enableVoltageCompensation(12);

        m_rightMotor.burnFlash();
        m_leftMotor.burnFlash();

    }

    @Override
    public void periodic() {

        TunableNumber.ifChanged(
            hashCode(), 
            pid -> setPID(
                pid[0], 
                pid[1], 
                pid[2]), 
            kP, 
            kI, 
            kD);

        TunableNumber.ifChanged(
        hashCode(), 
        kSVA -> m_ffController = new SimpleMotorFeedforward(
            kSVA[0], 
            kSVA[1], 
            kSVA[2]), 
        kS, 
        kV, 
        kA);

        TunableNumber.ifChanged(
        hashCode(),
        accel -> {
          leftProfile.setMaxAcceleration(accel[0]);
          rightProfile.setMaxAcceleration(accel[0]);
        },
        maxAcceleration);

        // Stop when disabled
        if (DriverStation.isDisabled()) {
        setGoal(Goal.IDLE);
        }

        // Check if profile needs to be reset
        if (!closedLoop && wasClosedLoop) {
        leftProfile.reset();
        rightProfile.reset();
        wasClosedLoop = false;
        }

        // Get goal
        double leftGoal = goal.getLeftGoalRpm().getAsDouble();
        double rightGoal = goal.getRightGoalRpm().getAsDouble();
        boolean idlePrepareShoot = goal == Goal.IDLE && prepareShootSupplier.getAsBoolean();
        if (idlePrepareShoot) {
        leftGoal = Goal.SHOOT.getLeftGoalRpm().getAsDouble() * prepareShootMultiplier.getAsDouble();
        rightGoal = Goal.SHOOT.getRightGoalRpm().getAsDouble() * prepareShootMultiplier.getAsDouble();
        }

        // Run to setpoint
        if (closedLoop || idlePrepareShoot) {
        // Update goals
        leftProfile.setGoal(leftGoal);
        rightProfile.setGoal(rightGoal);
        double leftSetpoint = leftProfile.calculateSetpoint();
        double rightSetpoint = rightProfile.calculateSetpoint();
        double leftPid = m_leftPidController.calculate(getLeftVelocityRpm(), leftSetpoint);
        double rightPid= m_rightPidController.calculate(getRightVelocityRpm(), rightSetpoint);
        runVoltage( leftPid + m_ffController.calculate(leftSetpoint), rightPid + m_ffController.calculate(rightSetpoint));
        } else if (goal == Goal.IDLE) {
        stop();
        }

        SmartDashboard.putString("Shooter Goal", getGoal().name());

    }

    public double getLeftVelocityRpm(){
        return m_leftEncoder.getVelocity();
    }

    public double getRightVelocityRpm(){
        return m_rightEncoder.getVelocity();
    }

    public void stop(){
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    public void runVoltage(double left, double right){
        m_leftMotor.setVoltage(left);
        m_rightMotor.setVoltage(right);
    }

    public void setGoal(Goal goal) {
        if (goal == Shooter.Goal.IDLE) {
          wasClosedLoop = closedLoop;
          closedLoop = false;
          this.goal = goal;
          return; // Don't set a goal
        }
        // If not already controlling to requested goal
        // set closed loop false
        closedLoop = this.goal == goal;
        // Enable close loop
        if (!closedLoop) {
          leftProfile.setGoal(goal.getLeftGoalRpm().getAsDouble(), getLeftVelocityRpm());
          rightProfile.setGoal(goal.getRightGoalRpm().getAsDouble(), getRightVelocityRpm());
          closedLoop = true;
        }
        this.goal = goal;
    }

    public boolean atGoal(){
        return goal == Goal.IDLE || (getLeftVelocityRpm() == goal.getLeftGoalRpm().getAsDouble() && getRightVelocityRpm() == goal.getRightGoalRpm().getAsDouble());
    }

    public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command ejectCommand() {
    return startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Eject");
  }
}
