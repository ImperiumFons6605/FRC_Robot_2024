package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ClimberConstants;

public class Climber extends ProfiledPIDSubsystem {
    
    
   

public Climber () {
    super(
        new ProfiledPIDController(
            ClimberConstants.kClimberP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ClimberConstants.kMaxVelocityRadPerSecond,
                ClimberConstants.kMaxAccelerationRadPerSecSquared)),
        0);

    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
}
