package frc.robot.subsystems.superstruct;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Shooter;
import frc.utils.ShootCalculator.ShotData;

public class Visualizer {
    
    private final Climber climber;
    private final Intake intake;
    private final Indexer indexer;
    private final Hood hood;
    private final Shooter shooter;

    private final Mechanism2d hoodMech = new Mechanism2d(3.0, 2.0, new Color8Bit(Color.kWhite));
    private final Mechanism2d climberMech = new Mechanism2d(2.0, 3.0, new Color8Bit(Color.kWhite));

    private final MechanismLigament2d hoodVisual;
    private final MechanismLigament2d climberVisual;

    private final FlywheelSim intakeSim = new FlywheelSim(
        DCMotor.getNEO(1),
        1,
        0.1, 
        VecBuilder.fill(0.01));
    private final FlywheelSim indexerSim = new FlywheelSim(
        DCMotor.getNeo550(1),
        1,
        0.1, 
        VecBuilder.fill(0.01));
    
    private final FlywheelSim shooterLeftSim = new FlywheelSim(
        DCMotor.getNEO(1),
        1,
        0.3, 
        VecBuilder.fill(0.01));
    
    private final FlywheelSim shooterRightSim = new FlywheelSim(
        DCMotor.getNEO(1),
        1,
        0.3, 
        VecBuilder.fill(0.01));
    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
        DCMotor.getNEO(1),
        HoodConstants.kHoodGearbox,
        0.2,
        0.3,
        Units.degreesToRadians(HoodConstants.kMinAngleDegrees), 
        Units.degreesToRadians(HoodConstants.kMaxAngleDegrees),
        true,
        0);
    private final ElevatorSim climberSim = new ElevatorSim(
        DCMotor.getNEO(2),
        ClimberConstants.kGearbox,
        0.5,
        1,
        ClimberConstants.kMinDistanceMeter,
        ClimberConstants.kMaxDistanceMeter,
        false,
        ClimberConstants.kMinDistanceMeter);
        


    public Visualizer(Climber climber, Intake intake, Indexer indexer, Hood hood, Shooter shooter){
        this.climber = climber;
        this.intake = intake;
        this.indexer= indexer;
        this.hood = hood;
        this.shooter = shooter;

        MechanismRoot2d hoodRoot = hoodMech.getRoot("hoodPivot", 1.0, 0.5);
        MechanismRoot2d climberRoot = climberMech.getRoot("ClimberRoot", 1.0, 0.5);

        climberVisual = new MechanismLigament2d("Climber", 0.4, 90, 6, new Color8Bit(Color.kBlue));
        hoodVisual = new MechanismLigament2d("Climber", 0.4, 0, 6, new Color8Bit(Color.kDarkGray));

        hoodRoot.append(hoodVisual);
        climberRoot.append(climberVisual);

        SmartDashboard.putData("Hood Mech", hoodMech);
        SmartDashboard.putData("Climber Mech", climberMech);

    }

    public void update(){
        hoodVisual.setAngle(hood.getMeasurement());
        climberVisual.setLength(climber.getMeasurement());
        SmartDashboard.putNumber("Intake Velocity", intake.getVelocityRPM());
        SmartDashboard.putNumber("Indexer Velocity", indexer.getVelocityRPM());
        SmartDashboard.putNumber("ShooterLeft Velocity", shooter.getLeftVelocityRpm());
        SmartDashboard.putNumber("ShooterRight Velocity", shooter.getRightVelocityRpm());
    }

    public void simulationUpdate(){
        hoodSim.setInput(hood.getVelocityDegPerSec());
        hoodSim.update(0.02);

        climberSim.setInput(climber.getVelocityMetersPerSec());
        hoodSim.update(0.02);

        intakeSim.setInputVoltage(intake.getGoal().getGoalVoltage().getAsDouble());
        intakeSim.update(0.02);

        indexerSim.setInputVoltage(indexer.getGoal().getGoalVoltage().getAsDouble());
        indexerSim.update(0.02);

        shooterLeftSim.setInput(shooter.getLeftVelocityRpm());
        shooterLeftSim.update(0.02);

        shooterRightSim.setInput(shooter.getRightVelocityRpm());
        shooterRightSim.update(0.02);
    }




}
