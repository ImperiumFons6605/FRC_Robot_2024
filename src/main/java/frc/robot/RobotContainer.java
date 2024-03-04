// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.GroundGrab;
import frc.robot.commands.ShootManual;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Climber m_climber = new Climber();
  private static final Shooter m_shooter = new Shooter();
  //private static final ShooterAngle m_shooterAngle = new ShooterAngle(new ProfiledPIDController(0, 0, 0, null));
  private static final Vision m_vision = new Vision(m_robotDrive);
  private static final Intake m_intake = new Intake();
  private static final Indexer m_indexer = new Indexer();
  private static final LEDs m_leds = new LEDs();

  //private int m_targetToTrack = 0;

  
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_subsystemsController = new CommandXboxController(OIConstants.kSubsytemsControllerPort);
  

 private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {

    autoChooser.addOption("straightTest", new PathPlannerAuto("straightAuto"));
    //autoChooser.addOption("strafeTest", new PathPlannerAuto("strafeAuto"));
    //autoChooser.addOption("turnTest", new PathPlannerAuto("turnAuto"));


    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));

    m_leds.setDefaultCommand(m_leds.setRainbow());
  }

 
  private void configureButtonBindings() {
    m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.setX()));
    m_driverController.x().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    //m_driverController.leftTrigger().whileTrue(new TurnToAprilTag(m_vision, m_robotDrive, m_driverController::getLeftY, m_driverController::getLeftX));
    m_driverController.rightTrigger().whileTrue(new GroundGrab(m_intake, m_indexer, m_leds));
    m_driverController.leftTrigger().whileTrue(new ShootManual(m_shooter));
    //m_driverController.povUp().onTrue(new InstantCommand(() -> m_targetToTrack = 3));
    //m_driverController.povRight().onTrue(new InstantCommand(() -> m_targetToTrack = 4));
    //m_driverController.povLeft().onTrue(new InstantCommand(() -> m_targetToTrack = 1));
    //m_driverController.povDown().onTrue(new InstantCommand(() -> m_targetToTrack = 0));

  }

  
  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
  }

  public void periodic(){
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //SmartDashboard.putBoolean("IRSensor reading", IRSensor.get());
  }

  public double pow(double input){

    double value = input * input;
    return (input < 0) ? -value : value;
  }
}