package frc.robot.subsystems.superstruct;

import java.time.Duration;
import java.util.spi.CurrencyNameProvider;

import javax.naming.spi.DirStateFactory;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.hal.DigitalGlitchFilterJNI;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DigitalPWMSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    
    public enum Goal{
        STOP,
        IDLE,
        AIM,
        INTAKE,
        EJECT,
        AMP_AIM,
        AMP_SHOOT,
        CLIMB,
        PREPARE_CLIMB,
        SHOOT,
        TRAP_AIM,
        TRAP_SHOOT
    }

    public enum GamePieceState {
        NONE,
        STAGED,
    }

    private Goal currentGoal = Goal.IDLE;
    private Goal desiredGoal = Goal.IDLE;
    private GamePieceState gamePieceState = GamePieceState.NONE;
    private GamePieceState lastGamePieceState = GamePieceState.NONE;
    private Timer gamepieceStateTimer = new Timer();

    private final Climber climber;
    private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Visualizer visualizer;

    private ColorSensorV3 distanceSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public boolean isNoteStaged(){
        return distanceSensor.getProximity() > 1000;
    }

    public SuperStructure(Climber climber, Hood hood, Indexer indexer, Intake intake, Visualizer visualizer){
        this.climber = climber;
        this.hood = hood;
        this.indexer = indexer;
        this.intake = intake;
        this.visualizer = visualizer;

        setDefaultCommand(setGoalCommand(Goal.IDLE));
        gamepieceStateTimer.start();


    }

    @Override
    public void periodic() {
        currentGoal =  desiredGoal;

        if(isNoteStaged()){
            gamePieceState = GamePieceState.STAGED;
        } else {
            gamePieceState = GamePieceState.NONE;
        }

        if(gamePieceState != lastGamePieceState){
            gamepieceStateTimer.reset();
        }

        lastGamePieceState = gamePieceState;

        //TODO add note Visualizer and update
        
        switch(currentGoal){
            case STOP -> {
                hood.setGoal(Hood.Goal.STOP);
                climber.setGoal(Climber.State.RETRACTING);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.IDLING);
            }
            case IDLE, CLIMB -> {
                hood.setGoal(Hood.Goal.STORE);
                climber.setGoal(Climber.State.RETRACTING);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.IDLING);
            }
            case INTAKE -> {
                if(!isNoteStaged()){
                    climber.setGoal(Climber.State.RETRACTING);
                    hood.setGoal(Hood.Goal.INTAKE);
                    if(hood.atGoal()){
                        intake.setGoal(Intake.Goal.INTAKING);
                        indexer.setGoal(Indexer.Goal.INTAKING);
                    }
                } else {
                    climber.setGoal(Climber.State.RETRACTING);
                    hood.setGoal(Hood.Goal.STORE);
                    intake.setGoal(Intake.Goal.IDLING);
                    indexer.setGoal(Indexer.Goal.IDLING);
                }
            }

            case EJECT -> {
                if(isNoteStaged()){
                    climber.setGoal(Climber.State.RETRACTING);
                    hood.setGoal(Hood.Goal.INTAKE);
                    if(hood.atGoal()){
                        intake.setGoal(Intake.Goal.EJECTING);
                        indexer.setGoal(Indexer.Goal.EJECTING);
                    }
                } else {
                    climber.setGoal(Climber.State.RETRACTING);
                    hood.setGoal(Hood.Goal.STORE);
                    intake.setGoal(Intake.Goal.IDLING);
                    indexer.setGoal(Indexer.Goal.IDLING);
                }
            }
            case AIM -> {
                climber.setGoal(Climber.State.RETRACTING);
                hood.setGoal(Hood.Goal.AIM);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.IDLING);
            }
            case SHOOT -> {
                climber.setGoal(Climber.State.RETRACTING);
                hood.setGoal(Hood.Goal.AIM);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.FEED_SHOOTER);
            }
            case AMP_AIM -> {
                climber.setGoal(Climber.State.RETRACTING);
                hood.setGoal(Hood.Goal.AMP);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.IDLING);
        
            }
            case AMP_SHOOT -> {
                climber.setGoal(Climber.State.RETRACTING);
                hood.setGoal(Hood.Goal.AMP);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.FEED_SHOOTER);
        
            }
            case PREPARE_CLIMB -> {
                climber.setGoal(Climber.State.EXTENDING);
                hood.setGoal(Hood.Goal.STORE);
                intake.setGoal(Intake.Goal.IDLING);
                indexer.setGoal(Indexer.Goal.IDLING);
            }
        }

        indexer.periodic();
        intake.periodic();
        visualizer.update(); 
        SmartDashboard.putString("Superstructure current Goal", getGoal().name());
        SmartDashboard.putBoolean("Note Staged", isNoteStaged());

    }

    @Override
    public void simulationPeriodic() {
        visualizer.simulationUpdate();
    }


    public void setGoal(Goal goal){
        if (desiredGoal == goal) return;
        desiredGoal = goal;
    } 


    public Command setGoalCommand(Goal goal){
        return startEnd(() -> setGoal(goal), () -> setGoal(Goal.IDLE))
        .withName("Superstrcture " + goal);
        
    }

    public boolean atGoal(){
        return currentGoal == desiredGoal && hood.atGoal() && climber.atGoal();
    }

    public SuperStructure.Goal getGoal(){
        return currentGoal;
    }

    public SuperStructure.GamePieceState getGamePieceState(){
        return gamePieceState;
    }


}
