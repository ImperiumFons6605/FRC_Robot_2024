// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean tuningMode = true;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);

    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kModuleTranslations);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    public static final int kPigeonGyroID = 10;

    public static final int kFrontLeftDrivingCanId = 9;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = false;


    public static final double headingkP = 1.0;
    public static final double headingkI = 0.0;
    public static final double headingkD = 0.0;
    public static final double headingkMaxVelRadPerSec = 4;
    public static final double headingkAccelRadPerSecSquared = 8;

    public static final double TranslationkP = 1;
    public static final double translationkI = 0.0;
    public static final double translationkD = 0.0;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.6;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSubsytemsControllerPort = 1;
    public static final double kDriveDeadband = 0.05;

    
    public static final int kLogitechLeftYAxis = 1;
    public static final int kLogitechLeftXAxis = 0;
    public static final int kLogitechRightYAxis = 3;
    public static final int kLogitechRightXAxis = 2;

    public static final int kLogitechDownButton= 2;
    public static final int kLogitechRightButton= 3;
    public static final int kLogitechLeftButton= 1;
    public static final int kLogitechUpButton= 4;
    public static final int kLogitechR1= 6;
    public static final int kLogitechL1= 5;
    public static final int kLogitechR2= 7;
    public static final int kLogitechL2= 8;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }


  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  
  public static final class ClimberConstants {
    public static final double sproketPitchDiameter = 0.032766; //pitch diameter in m
    
    public static final double kGearbox = (10.0/52.0)*(30.0/68.0);
    public static final double kEncoderPositionFactor = Math.PI * sproketPitchDiameter; // m
    public static final double kEncoderVelocityFactor = kEncoderPositionFactor /60.0; // m/s

    //TODO Change Hood zero offset, calibrate in Rev Hardware
    public static final double kEncoderZeroOffset = 0.0;
    public static final boolean kEncoderInverted = false;

    public static final double kMaxVelocityMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecSquared = 0;

    public static final int kClimberMasterCANId = 13;
    public static final int kClimberSlaveCANId = 14;

    public static final double kClimberP = 0.6;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;

    public static final double kMinDistanceMeter = Units.inchesToMeters(2);
    public static final double kMaxDistanceMeter = Units.inchesToMeters(15.5);
    
  }

  public static final class IntakeConstants{
    public static final int kIntakeID = 11;
    public static final double kIntakePot = 0.9;
  }

  public static final class IndexerConstants{
    public static final int kIndexerID = 12;
    public static final double kIndexerGrabPot = 0.7;
    public static final double kIndexerShootPot = 1;
  }

  public static final class ShooterConstants{
    public static final int kLeftID = 15;
    public static final int kRightID = 16;

    public static final double kShooterP = 0.2;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.001;

    public static final double ShooterKS = 0.8;
    public static final double ShooterKV = 0.001;
    public static final double ShooterKA = 0.0;

    public static final double ShooterLeftRpm = 5600.0;
    public static final double ShooterRightRpm = 3800.0;

    public static final double prepareShootMultiplier = 0.75;
    public static final double EjectingRpm = 1000.0;
    public static final double maxAccelerationRpmPerSec = 9000.0;

    public static final double kGearboxReduction = 1.0;
    public static final double kVelocityEncoderFactor = 1.0 / kGearboxReduction; //Medido en RPMs
    
  }

  public static final class HoodConstants{

    public static final int kHoodID = 17;

    public static final double kHoodGearbox = 418.0/30.0; //Hood Gear / Pinion Gear teeth

    public static final double kHoodEncoderPositionFactor = 360.0 / kHoodGearbox; //1 vuelta == 360°
    public static final double kHoodEncoderVelocityFactor = kHoodEncoderPositionFactor/ 60.0; //1 vuelta == 360°

    //TODO Change Hood zero offset, calibrate in Rev Hardware
    public static final double kEncoderZeroOffset = 0.0;
    public static final boolean kEncoderInverted = false;

    public static final double kHoodP = 0.01;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;

    public static final double HoodKS = 0.0;
    public static final double HoodkG = 0.36;
    public static final double HoodKV = 0.1;

    public static final double kMaxVelocityDegPerSec = 90.0;
    public static final double kMaxAccelDegPerSecSquared = 180.0;

    public static final double kMinAngleDegrees = 0.0;
    public static final double kMaxAngleDegrees = 50.0;
    
  }

  public static final class VisionConstants{

    public static boolean kPoseEstimatorEnabled = false;
    public static final Transform3d kTransformRobotToShooterCam = new Transform3d(
      new Translation3d(0.2, 0.22, 0.23),//0.23),
      new Rotation3d(
              Units.degreesToRadians(180), Units.degreesToRadians(-45),
              0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    public static final Transform3d kTransformRobotToIntakeCam = new Transform3d(
      new Translation3d(-0.2, -0.22, 0.23),//0.23),
      new Rotation3d(
              Units.degreesToRadians(180), Units.degreesToRadians(45),
              Units.degreesToRadians(180))); // Cam mounted facing forward, half a meter forward of center, half a meter up
// from center.
    public static final String kCameraName1 = "6605Camera1";
    public static final String kCameraName2 = "6605Camera2";

    public static final double kXControllerP = 0.13;
    public static final double kXControllerD = 0;
    public static final double kYControllerP = 0.21;
    public static final double kYControllerD = 0;
    public static final double kOmegaControllerP = 0.22;
    public static final double kOmegaControllerD = 0;

    public static final double kMaxXVelocity = 2;
    public static final double kMaxXAcceleration = 3;

    public static final double kMaxYVelocity = 2;
    public static final double kMaxYAcceleration = 3;

    public static final double kMaxOmegaVelocity = 1;
    public static final double kMaxOmegaAcceleration = 1.5;
  }

  public static final class FieldConstants{
    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);

    public static final Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

    /** Staging locations for each note */
    public static final class StagingLocations {
      public static final double centerlineX = fieldLength / 2.0;

      // need to update
      public static final double centerlineFirstY = Units.inchesToMeters(29.638);
      public static final double centerlineSeparationY = Units.inchesToMeters(66);
      public static final double spikeX = Units.inchesToMeters(114);
      // need
      public static final double spikeFirstY = Units.inchesToMeters(161.638);
      public static final double spikeSeparationY = Units.inchesToMeters(57);

      public static final Translation2d[] centerlineTranslations = new Translation2d[5];
      public static final Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }

      public static final double chainMinHeight = Units.inchesToMeters(28.25);
    }

    /** Each corner of the speaker * */
    public static final class Speaker {

      // corners (blue alliance origin)
      public static final Translation3d topRightSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(238.815),
              Units.inchesToMeters(83.091));

      public static final Translation3d topLeftSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(197.765),
              Units.inchesToMeters(83.091));

      public static final Translation3d bottomRightSpeaker =
          new Translation3d(
            0.0, 
            Units.inchesToMeters(238.815), 
            Units.inchesToMeters(78.324));

      public static final Translation3d bottomLeftSpeaker =
          new Translation3d(
            0.0, 
            Units.inchesToMeters(197.765), 
            Units.inchesToMeters(78.324));

      /** Center of the speaker opening (blue alliance) */
      public static final Translation3d centerSpeakerOpening =
          bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
    }

    public static final class Subwoofer {
      public static final Pose2d ampFaceCorner =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(239.366),
              Rotation2d.fromDegrees(-120));

      public static final Pose2d sourceFaceCorner =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(197.466),
              Rotation2d.fromDegrees(120));

      public static final Pose2d centerFace =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(218.416),
              Rotation2d.fromDegrees(180));
    }

    public static final class Stage {
      public static final Pose2d podiumLeg =
          new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());
      public static final Pose2d ampLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(212.425),
              Rotation2d.fromDegrees(-30));
      public static final Pose2d sourceLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(110.837),
              Rotation2d.fromDegrees(30));

      public static final Pose2d centerPodiumAmpChain =
          new Pose2d(
              podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(120.0));
      public static final Pose2d centerAmpSourceChain =
          new Pose2d(
              ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
      public static final Pose2d centerSourcePodiumChain =
          new Pose2d(
              sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(240.0));
      public static final Pose2d center =
          new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
      public static final double centerToChainDistance =
          center.getTranslation().getDistance(centerPodiumAmpChain.getTranslation());
    }

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  }

}