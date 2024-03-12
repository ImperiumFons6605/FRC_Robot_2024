// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.superstruct.Climber;
import frc.robot.subsystems.superstruct.Hood;
import frc.robot.subsystems.superstruct.Indexer;
import frc.robot.subsystems.superstruct.Intake;
import frc.robot.subsystems.superstruct.SuperStructure;
import frc.robot.subsystems.superstruct.Visualizer;
import frc.robot.subsystems.superstruct.SuperStructure.Goal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Climber m_climber = new Climber();
  private static final Vision m_vision = new Vision(m_robotDrive);
  private static final Intake m_intake = new Intake();
  private static final Indexer m_indexer = new Indexer();
  private static final Hood m_hood = new Hood();
  private static final Shooter m_shooter = new Shooter();
  private static final Visualizer m_visualizer = new Visualizer(m_climber, m_intake, m_indexer, m_hood, m_shooter);

  private static final LEDs m_leds = new LEDs();

  private static final SuperStructure m_superstructure = new SuperStructure(m_climber, m_hood, m_indexer, m_intake, m_visualizer);


  
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_subsystemsController = new CommandXboxController(OIConstants.kSubsytemsControllerPort);
  

 
private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
  public RobotContainer() {

    autoChooser.addOption("straightTest", new PathPlannerAuto("straightAuto"));
    autoChooser.addOption("6 Note Auto", new PathPlannerAuto("6 note Auto"));

    configureButtonBindings();
    
    if(RobotBase.isReal()){
      m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.accepTeleopInput(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
    }else {
      m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.accepTeleopInput(
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
    }
    m_leds.setDefaultCommand(m_leds.setRainbow());

    Function<Double, Command> controllerRumbleCommandFactory =
        time ->
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                      m_subsystemsController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                      m_leds.endgameAlert = true;
                    }),
                Commands.waitSeconds(time),
                Commands.runOnce(
                    () -> {
                      m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      m_subsystemsController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      m_leds.endgameAlert = false;
                    }));

    new Trigger(() ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round((30.0)))
        .onTrue(controllerRumbleCommandFactory.apply(0.5));
    new Trigger(() ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(15.0))
        .onTrue(
            Commands.sequence(
                controllerRumbleCommandFactory.apply(0.2),
                Commands.waitSeconds(0.1),
                controllerRumbleCommandFactory.apply(0.2),
                Commands.waitSeconds(0.1),
                controllerRumbleCommandFactory.apply(0.2)));
  }

 
  private void configureButtonBindings() {
    m_driverController.rightTrigger().whileTrue(m_superstructure.setGoalCommand(Goal.INTAKE));
    m_driverController.a().whileTrue(m_superstructure.setGoalCommand(Goal.AIM));
    m_driverController.leftTrigger()
    .and(
      new Trigger(m_shooter::atGoal))
      .whileTrue(
        Commands.deadline(
          Commands.race(new WaitCommand(1), Commands.waitUntil(m_driverController.a().negate())),
           m_shooter.shootCommand()));

  }

  
  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
  }

  public void periodic(){
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Hood Measurement", m_hood.getMeasurementDegrees());
  }

  public double pow(double input){

    double value = input * input;
    return (input < 0) ? -value : value;
  }
  
}