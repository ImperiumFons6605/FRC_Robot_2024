package frc.robot.commands;

import java.lang.constant.DirectMethodHandleDesc;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class AlignApriltag extends Command {

    
    private final TrapezoidProfile.Constraints XConstraints = new Constraints(
        VisionConstants.kMaxXVelocity, 
        VisionConstants.kMaxXAcceleration);
    private final TrapezoidProfile.Constraints YConstraints = new Constraints(
        VisionConstants.kMaxYVelocity, 
        VisionConstants.kMaxYAcceleration);
    private final TrapezoidProfile.Constraints OmegaConstraints = new Constraints(
        VisionConstants.kMaxOmegaVelocity, 
        VisionConstants.kMaxOmegaAcceleration);

    private static final Transform3d tagGoal = 
        new Transform3d(
            new Translation3d(0.5,0,0), 
            new Rotation3d(0, 0, Math.PI)
        );

    private final PhotonCamera m_camera;
    private final DriveSubsystem m_driveSubsystem;
    private final Supplier<Pose2d> m_robotPoserProvider;
    private int tagToChase = 1;
    private int m_GoalToChase = 1;
    private PhotonTrackedTarget m_lastTarget;
    private boolean[] isFinished;

    private final ProfiledPIDController xController = 
        new ProfiledPIDController(
            VisionConstants.kXControllerP, 
            0, 
            VisionConstants.kXControllerD, XConstraints);

    private final ProfiledPIDController yController = 
        new ProfiledPIDController(
            VisionConstants.kYControllerP, 
            0, 
            VisionConstants.kYControllerD, YConstraints);

    private final ProfiledPIDController omegaController = 
        new ProfiledPIDController(
            VisionConstants.kOmegaControllerP, 
            0, 
            VisionConstants.kOmegaControllerD, OmegaConstraints);

    public AlignApriltag(Vision vision, DriveSubsystem drive){
        isFinished =  new boolean[3];
        isFinished[0] = false;
        isFinished[1] = false;
        isFinished[2] = false;
        m_camera = vision.getCamera();
        m_driveSubsystem = drive;
        m_robotPoserProvider = drive::getPose;

        xController.setTolerance(0.04);
        yController.setTolerance(0.04);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (m_camera.getLatestResult().hasTargets()) {
            tagToChase =  m_camera.getLatestResult().getBestTarget().getFiducialId();
        } else {
            isFinished[0] = true;
            isFinished[1] = true;
            isFinished[2] = true;
        }
        var robotPose = m_robotPoserProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        xController.reset(robotPose.getY());
        var cameraRes = m_camera.getLatestResult();
        if (cameraRes.hasTargets()) {
            m_lastTarget = cameraRes.getBestTarget();
            tagToChase =  m_lastTarget.getFiducialId();
        } else {
            m_lastTarget = null;
        }
        
    }

    @Override
    public void execute() {
        var robotPose2d = m_robotPoserProvider.get();
        var robotPose3d = new Pose3d(robotPose2d);

        var cameraRes = m_camera.getLatestResult();
        if (cameraRes.hasTargets()) {
            var targetOpt = cameraRes.getTargets().stream()
                .filter(t -> t.getFiducialId() == tagToChase) 
                .filter(t -> t.equals(m_lastTarget))
                .findFirst();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                m_lastTarget = target;
                var cameraPose = robotPose3d.transformBy(VisionConstants.kTransformRobotToCam);
                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);
                var goalPose = targetPose.transformBy(tagGoal).toPose2d();
                /* 
                SmartDashboard.putNumber("goal X", goalPose.getX());
                SmartDashboard.putNumber("goal Y", goalPose.getY());
                SmartDashboard.putNumber("goal Omega", goalPose.getRotation().getRadians());
                */
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());
                SmartDashboard.putNumber("GoalX", goalPose.getX());
                SmartDashboard.putNumber("GoalY", goalPose.getY());
                SmartDashboard.putNumber("GoalTheta", goalPose.getRotation().getRadians());
            }
        }
        
        if (m_lastTarget == null) {
            m_driveSubsystem.drive(0, 0, 0, false, false);
        } else {
            var xSpeed = xController.calculate(robotPose2d.getX());
            if (xController.atGoal()){
                xSpeed = 0;
                isFinished[0] = true;
            }
            var ySpeed = yController.calculate(robotPose2d.getY());
            if (yController.atGoal()){
                ySpeed = 0;
                isFinished[1] = true;
            }
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()){
                omegaSpeed = 0;
                isFinished[2] = true;
            }
            m_driveSubsystem.drive(xSpeed, ySpeed, omegaSpeed, true, false);
            SmartDashboard.putNumber("VisionSpeedX", xSpeed);
            SmartDashboard.putNumber("VisionSpeedY", ySpeed);
            SmartDashboard.putNumber("VisionSpeedOmega", omegaSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false, false);
    } 
    @Override
    public boolean isFinished() {
        if((isFinished[0] = true) && (isFinished[1] = true) && (isFinished[2] = true)){
            return true;
        }
        return false;
    }
}
