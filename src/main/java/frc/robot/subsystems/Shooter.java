package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    BangBangController bang = new BangBangController();

    public Shooter(){
        
    }
}
