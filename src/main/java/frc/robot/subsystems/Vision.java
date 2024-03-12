package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.SimVisionSystem;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{

    private PhotonCamera m_cameraShooter;
    private PhotonCamera m_cameraIntake;
    private DriveSubsystem drive;
    private PhotonPoseEstimator m_photonPoseEstimatorShooter;
    private PhotonPoseEstimator m_photonPoseEstimatorIntake;

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

    PhotonCameraSim cameraShooterSim;
    PhotonCameraSim cameraIntakeSim;
    SimCameraProperties simPiCamV2Prop = new SimCameraProperties();

    VisionSystemSim simVision = new VisionSystemSim("photonSystemSim");
    
    public Vision(DriveSubsystem driveP){
        drive = driveP;
        m_cameraShooter = new PhotonCamera(VisionConstants.kCameraName1);
        m_cameraIntake = new PhotonCamera(VisionConstants.kCameraName2);

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            m_photonPoseEstimatorShooter =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cameraShooter, VisionConstants.kTransformRobotToShooterCam);
            m_photonPoseEstimatorShooter.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

            m_photonPoseEstimatorShooter =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cameraShooter, VisionConstants.kTransformRobotToIntakeCam);
            m_photonPoseEstimatorShooter.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            m_photonPoseEstimatorShooter = null;
        }

        if(RobotBase.isSimulation()){
            // A 640 x 480 camera with a 100 degree diagonal FOV.
            simPiCamV2Prop.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            simPiCamV2Prop.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            simPiCamV2Prop.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            simPiCamV2Prop.setAvgLatencyMs(35);
            simPiCamV2Prop.setLatencyStdDevMs(5);

            cameraShooterSim = new PhotonCameraSim(m_cameraShooter, simPiCamV2Prop);
            cameraIntakeSim = new PhotonCameraSim(m_cameraIntake, simPiCamV2Prop);
            simVision.addCamera(cameraShooterSim, VisionConstants.kTransformRobotToShooterCam);
            simVision.addCamera(cameraIntakeSim, VisionConstants.kTransformRobotToIntakeCam);
            cameraIntakeSim.enableDrawWireframe(false);
            cameraShooterSim.enableDrawWireframe(false);

            try {
                AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                simVision.addAprilTags(fieldLayout);

            } catch (IOException e) {
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                simVision = null;
            }
        }
    }
    
    public Optional<Pose3d> getApritagPose(int id) {
        var res = m_cameraShooter.getLatestResult();
        if (res.hasTargets()) {
            var target = res.getTargets().get(id);
            var poseAmbiguity = target.getPoseAmbiguity();
            SmartDashboard.putNumber("TargetPoseAMbiguity", poseAmbiguity);
            if(poseAmbiguity == -1 || poseAmbiguity > 0.4){
                return Optional.empty();
            } else {
                return Optional.of(new Pose3d(drive.getPose()).transformBy(VisionConstants.kTransformRobotToShooterCam).transformBy(target.getBestCameraToTarget()));
            }  
        } else {
            return Optional.empty();
        }
    }

    public PhotonCamera getCamera(){
        return m_cameraShooter;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalShooterCameraPose(Pose2d prevEstimatedRobotPose) {
        if (m_photonPoseEstimatorShooter == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        m_photonPoseEstimatorShooter.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimatorShooter.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalIntakeCameraPose(Pose2d prevEstimatedRobotPose) {
        if (m_photonPoseEstimatorIntake == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        m_photonPoseEstimatorIntake.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimatorIntake.update();
    }


    @Override
    public void periodic() {
        if(VisionConstants.kPoseEstimatorEnabled){
            var estimatedRobotPoseShooterCamera = getEstimatedGlobalShooterCameraPose(drive.getPose());
            var estimatedRobotPoseIntakeCamera = getEstimatedGlobalIntakeCameraPose(drive.getPose());

            if(estimatedRobotPoseShooterCamera.isPresent()){
                drive.addVisionMeasurement("CameraShooter",estimatedRobotPoseShooterCamera.get().estimatedPose.toPose2d(), estimatedRobotPoseShooterCamera.get().timestampSeconds);
            }

            if(estimatedRobotPoseIntakeCamera.isPresent()){
                drive.addVisionMeasurement("Camera Intake", estimatedRobotPoseIntakeCamera.get().estimatedPose.toPose2d(), estimatedRobotPoseIntakeCamera.get().timestampSeconds);
            }

            if(RobotBase.isSimulation()){
                //SmartDashboard.putData(simVision.getDebugField());
            }
        }
    }
}
