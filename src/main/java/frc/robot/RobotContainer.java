// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem; 
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer {
  
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();

  
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

 private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  private final Field2d field;

  public RobotContainer() {

    autoChooser.addOption("straightTest", new PathPlannerAuto("FULLFront"));
    autoChooser.addOption("rightTest", new PathPlannerAuto("FULLRight"));
    autoChooser.addOption("90degreeTest", new PathPlannerAuto("rotation 90"));
     
            field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
   });


    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));



    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

 
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

  }

  
  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
  }


  public void sendTelemetry(){
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putNumber("controlX", m_driverController.getLeftX());
    SmartDashboard.putNumber("controlY", m_driverController.getLeftY());
    SmartDashboard.putNumber("controlRot", m_driverController.getRightX());
  }




}