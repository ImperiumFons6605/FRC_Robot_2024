package frc.robot.simulation;

import java.io.ObjectInputStream.GetField;
import java.lang.reflect.Field;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FieldVisualizer {
  private final DriveSubsystem m_swerveDrive;
  private final Field2d field = new Field2d();

  private Pose2d[] m_swerveModulePoses = {
          new Pose2d(),
          new Pose2d(),
          new Pose2d(),
          new Pose2d()
  };

  public FieldVisualizer(DriveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
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
  }

  public Field2d GetField(){
    return field;
  }

  public void updateRobotPoses() {
    field.setRobotPose(m_swerveDrive.getPose());

    for (int i = 0; i < DriveConstants.kModuleTranslations.length; i++) {
      Translation2d updatedPositions =
        DriveConstants.kModuleTranslations[i]
                      .rotateBy(m_swerveDrive.getPose().getRotation())
                      .plus(m_swerveDrive.getPose().getTranslation());
      m_swerveModulePoses[i] =
              new Pose2d(
                      updatedPositions,
                      m_swerveDrive
                              .getModules()[i]
                              .getPosition().angle
                              .plus(m_swerveDrive.getPose().getRotation()));
    }

    field.getObject("Swerve Modules").setPoses(m_swerveModulePoses);

    SmartDashboard.putData(field);
  }

}
