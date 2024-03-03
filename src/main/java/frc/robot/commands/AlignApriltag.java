package frc.robot.commands;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class AlignApriltag extends Command {

    private Vision visionSubsystem;
    private DriveSubsystem drive;
    private int idToTrack;
    private Pose3d targetPose;

    private Pose2d goalPose = new Pose2d(0.5, 0, new Rotation2d(Units.degreesToRadians(0)));

    private PIDController xController = new PIDController(0.1, 0, 0);
    private PIDController yController = new PIDController(0.1, 0, 0);
    private PIDController thetaController = new PIDController(0.1, 0, 0);


    public AlignApriltag(Vision vision, DriveSubsystem driveP, int id){
        visionSubsystem = vision;
        idToTrack = id;
        drive = driveP;
        addRequirements(driveP);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        thetaController.setSetpoint(goalPose.getRotation().getRadians());
        
    }
    @Override
    public void execute() {
        if(visionSubsystem.getApritagPose(idToTrack).isPresent()){
            SmartDashboard.putBoolean("isTargetPresent", true);
            targetPose = visionSubsystem.getApritagPose(idToTrack).get();
            SmartDashboard.putNumber("TargetX", targetPose.getX());
            SmartDashboard.putNumber("TargetY", targetPose.getY());
            SmartDashboard.putNumber("TargetZ", targetPose.getZ());
            SmartDashboard.putNumber("TargetRot", Units.radiansToDegrees(targetPose.getRotation().getAngle()));
            SmartDashboard.putNumber("DistanceToTarget", new Translation2d(targetPose.getTranslation().getX(), targetPose.getTranslation().getY()).getDistance(drive.getPose().getTranslation()));

            double xValue = xController.calculate(targetPose.getX());
            double yValue = yController.calculate(targetPose.getY());
            double thetaValue = -thetaController.calculate(targetPose.getRotation().getAngle());

            drive.drive(xValue, yValue, thetaValue, true, false);
        } else {
            SmartDashboard.putBoolean("isTargetPresent", false);
            SmartDashboard.putNumber("TargetX", 0);
            SmartDashboard.putNumber("TargetY", 0);
            SmartDashboard.putNumber("TargetZ", 0);
            SmartDashboard.putNumber("TargetRot", 0);
            SmartDashboard.putNumber("DistanceToTarget", 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
