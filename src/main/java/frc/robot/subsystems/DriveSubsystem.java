// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.simulation.FieldVisualizer;
import frc.utils.SwerveUtils;
import frc.utils.TunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase{

  public enum Mode{
      TELEOP,
      ALIGN_POSE,
      CHARACTERIZATION,
      AUTO
  }
  
  private DriveSubsystem.Mode currentMode = DriveSubsystem.Mode.TELEOP;

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  MAXSwerveModule[] SwerveModules = new MAXSwerveModule[]{m_frontLeft,m_frontRight,m_rearLeft,m_rearRight};

  // The gyro sensor
  private final Pigeon2 pigeon;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_poseEstimator;

  private final FieldVisualizer m_field = new FieldVisualizer(this);

  private boolean overrideHeading = false;
  private Rotation2d headingTarget = new Rotation2d();

  private TunableNumber headingkP = new TunableNumber("DriveSubsystem/HeadingkP", DriveConstants.headingkP);
  private TunableNumber headingkI = new TunableNumber("DriveSubsystem/HeadingkI", DriveConstants.headingkI);
  private TunableNumber headingkD = new TunableNumber("DriveSubsystem/HeadingkD", DriveConstants.headingkD);
  private TunableNumber headingkMaxVel = new TunableNumber("DriveSubsystem/HeadingkMaxVel", DriveConstants.headingkMaxVelRadPerSec);
  private TunableNumber headingkMaxAccel = new TunableNumber("DriveSubsystem/HeadingkMaxAccel", DriveConstants.headingkAccelRadPerSecSquared);

  private TunableNumber translationkP = new TunableNumber("DriveSubsystem/TranslationkP", DriveConstants.TranslationkP);
  private TunableNumber translationkI = new TunableNumber("DriveSubsystem/TranslationkP", DriveConstants.translationkI);
  private TunableNumber translationkD = new TunableNumber("DriveSubsystem/TranslationkP", DriveConstants.translationkD);
  private TunableNumber translationkMaxVel = new TunableNumber("DriveSubsystem/TranslationkMaxVel", DriveConstants.headingkMaxVelRadPerSec);
  private TunableNumber translationMaxAccel = new TunableNumber("DriveSubsystem/TranslationkMaxAccel", DriveConstants.headingkAccelRadPerSecSquared);

  private double xSpeeds = 0.0;
  private double ySpeeds = 0.0;
  private double omegaSpeeds = 0.0;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  private ProfiledPIDController headingController = new ProfiledPIDController(
    headingkP.getAsDouble(),
    headingkI.getAsDouble(),
    headingkD.getAsDouble(),
    new TrapezoidProfile.Constraints(
      headingkMaxVel.getAsDouble(),
      headingkMaxAccel.getAsDouble())
  );
  private void setHeadingPID(double p, double i, double d){
    headingController.setP(p);
    headingController.setI(i);
    headingController.setD(d);
  }
  private void setHeadingConstraint(double maxVel, double maxAccel){
    headingController.setConstraints(
      new TrapezoidProfile.Constraints(
        maxVel, 
        maxAccel));
  }

  private PIDController xController = new PIDController(
    translationkP.getAsDouble(), 
    translationkI.getAsDouble(),
    translationkD.getAsDouble()
  );

  private PIDController yController = new PIDController(
    translationkP.getAsDouble(), 
    translationkI.getAsDouble(),
    translationkD.getAsDouble()
  );

  private void setTranslationPID(double p, double i, double d){
    xController.setP(p);
    xController.setI(i);
    xController.setD(d);
    yController.setP(p);
    yController.setI(i);
    yController.setD(d);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometryTraj, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );


    Pathfinding.setPathfinder(new LocalADStar());

    pigeon = new Pigeon2(DriveConstants.kPigeonGyroID);

    /* Configure Pigeon2 */
    var toApply = new Pigeon2Configuration();

    /* User can change the configs if they want, or leave it empty for factory-default */

    pigeon.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    pigeon.getYaw().setUpdateFrequency(100);
    pigeon.getGravityVectorZ().setUpdateFrequency(100);


    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      new Rotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, 
      new Pose2d());

      pigeon.reset();

      headingController.enableContinuousInput(-Math.PI, Math.PI);

      //Logger.recordOutput("MyStates", getModuleStates());
      //Logger.recordOutput("MyPose2D", getPose());
  }

  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_poseEstimator.update(
        getHeadingRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    sendTelemetry();

    TunableNumber.ifChanged(
      hashCode(),
      headingPid -> {
        setHeadingPID(
        headingPid[0],
        headingPid[1],
        headingPid[2]);
      }, 
      headingkP,
      headingkI,
      headingkD);

    TunableNumber.ifChanged(
      hashCode(),
      headingContraints -> {
        setHeadingConstraint(
        headingContraints[0],
        headingContraints[1]);
      }, 
      headingkMaxVel,
      headingkMaxAccel);
    
    TunableNumber.ifChanged(
      hashCode(),
      translationPid -> {
        setTranslationPID(
        translationPid[0],
        translationPid[1],
        translationPid[2]);
      }, 
      translationkP,
      translationkI,
      translationkD);

    switch (currentMode) {
      case TELEOP -> {
        if(overrideHeading){
          double omega = headingController.calculate(getPose().getRotation().getRadians());
          if(Math.abs(omega) < 0.1){
            omega = 0.0;
          }
          desiredSpeeds.omegaRadiansPerSecond = omega;
        }
      }
      case ALIGN_POSE -> {

      }
      case CHARACTERIZATION -> {

      }
      default -> {}
      
    }

    if(currentMode != DriveSubsystem.Mode.AUTO || currentMode != DriveSubsystem.Mode.CHARACTERIZATION){
      SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      int i = 0;
      for(SwerveModuleState module: swerveModuleStates){
        i++;
      }
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    
  }

  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void accepTeleopInput(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    currentMode = Mode.TELEOP;
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    desiredSpeeds = 
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public void resetOdometryTraj(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getHeadingRotation2d(),
        getModulePositions(),
        pose);
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getHeadingRotation2d(),
        getModulePositions(),
        pose);
  
  }

   /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }
  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(String tag, Pose2d visionPose, double timestamp) {
    m_field.GetField().getObject(tag + " VisionRobotPose").setPose(visionPose);
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public Optional<Rotation2d> getTargetHeading(){
    if(overrideHeading){
      return Optional.of(headingTarget);
    } else {
      return Optional.empty();
    }
  }

  public void setOverrideHeading(boolean override){
    overrideHeading = override;
    headingController.reset(getPose().getRotation().getRadians());
  }

  public void setTargetHeading(Rotation2d heading){
    headingTarget = heading;
    headingController.setGoal(heading.getRadians());
  }

  //TODO Add pose ALign 
  public void setAutoAlignGoal(Supplier<Pose2d> poseSupplier, boolean slowMode){
    if(DriverStation.isTeleopEnabled()){
      currentMode = DriveSubsystem.Mode.ALIGN_POSE;
      
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon.reset();
  }

  public Rotation2d getHeadingRotation2d() {
    return pigeon.getRotation2d();
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return pigeon.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    accepTeleopInput(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);
  }

  /**
   * 
   * @return Robot Relative Chassis Speeds
   */
  public ChassisSpeeds getRobotSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      SwerveModules[0].getState(),
      SwerveModules[1].getState(),
      SwerveModules[2].getState(),
      SwerveModules[3].getState()
    };
  }

  
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { 
      SwerveModules[0].getPosition(),
      SwerveModules[1].getPosition(),
      SwerveModules[2].getPosition(),
      SwerveModules[3].getPosition()
    };
  }


  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }



  public MAXSwerveModule[] getModules() {
      return SwerveModules;
  }
  
  @Override
  public void simulationPeriodic() {

    m_field.updateRobotPoses();

    double m_simYaw = getRobotSpeeds().omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    pigeon.getSimState().addYaw(Units.radiansToDegrees(m_simYaw));

  }

  public void sendTelemetry(){
      SmartDashboard.putData(pigeon);
      int i = 0;
      for(MAXSwerveModule module: SwerveModules){
        i++;
      }

    m_field.updateRobotPoses();
    
  }



  
}
