package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimVisionSystem;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{

    private PhotonCamera m_camera;
    private DriveSubsystem drive;
    private PhotonPoseEstimator m_photonPoseEstimator;

    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 75.0; // degrees
    double camPitch = 15.0; // degrees
    double camHeightOffGround = 0.85; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    PhotonCameraSim simCamera;

    VisionSystemSim simVision = new VisionSystemSim("photonSystemSim");
    
    public Vision(DriveSubsystem driveP){
        m_camera = new PhotonCamera("photonvision");
        simCamera = new PhotonCameraSim(m_camera);
        drive = driveP;
        m_camera.setDriverMode(true);
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            m_photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, VisionConstants.kTransformRobotToCam);
            m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            m_photonPoseEstimator = null;
        }
        m_camera.setDriverMode(false);
        m_photonPoseEstimator.setRobotToCameraTransform(VisionConstants.kTransformRobotToCam);
        
        
    }
    public Optional<Pose3d> getApritagPose(int id) {
        var res = m_camera.getLatestResult();
        if (res.hasTargets()) {
            var target = res.getTargets().get(id);
            var poseAmbiguity = target.getPoseAmbiguity();
            SmartDashboard.putNumber("TargetPoseAMbiguity", poseAmbiguity);
            if(poseAmbiguity == -1 || poseAmbiguity > 0.3){
                return Optional.empty();
            } else {
                return Optional.of(new Pose3d().transformBy(VisionConstants.kTransformRobotToCam).transformBy(target.getBestCameraToTarget()));
            }  
        } else {
            return Optional.empty();
        }
    }

    public PhotonCamera getCamera(){
        return m_camera;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (m_photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }

    @Override
    public void periodic() {
        if(VisionConstants.kPoseEstimatorEnabled){
            var estimatedRobotPose = getEstimatedGlobalPose(drive.getPose());

            if(estimatedRobotPose.isPresent()){
                SmartDashboard.putNumber("VisionX", estimatedRobotPose.get().estimatedPose.toPose2d().getX());
                SmartDashboard.putNumber("VisionY", estimatedRobotPose.get().estimatedPose.toPose2d().getY());
                SmartDashboard.putNumber("VisionTheta", estimatedRobotPose.get().estimatedPose.toPose2d().getRotation().getDegrees());
                Logger.recordOutput("VisionEstimatedPose", estimatedRobotPose.get().estimatedPose.toPose2d());
                drive.addVisionMeasurement(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds);
            }
        }
    }
}
