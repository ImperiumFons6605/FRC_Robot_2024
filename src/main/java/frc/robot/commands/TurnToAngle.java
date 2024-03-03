package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class TurnToAngle extends Command{
    private Vision visionSubsystem;
    private DriveSubsystem drive;
    private int idToTrack;
    private Pose3d targetPose;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    private Pose2d goalPose = new Pose2d(0.5, 0, new Rotation2d(Units.degreesToRadians(0)));

    private PIDController xController = new PIDController(0.1, 0, 0);
    private PIDController yController = new PIDController(0.1, 0, 0);
    private PIDController thetaController = new PIDController(0.1, 0, 0);


    public TurnToAngle(Vision vision, DriveSubsystem driveP, DoubleSupplier x, DoubleSupplier y){
        visionSubsystem = vision;
        drive = driveP;
        xSupplier = x;
        ySupplier = y;
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
        if(visionSubsystem.getCamera().getLatestResult().hasTargets()){

            double thetaValue = -thetaController.calculate(visionSubsystem.getCamera().getLatestResult().getBestTarget().getYaw(), 0);
            drive.driveRobotRelative(new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaValue));
        } else {
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
