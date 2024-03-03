package frc.robot.subsystems;




import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    Spark ledSpark;

    
    public LEDs (){
    ledSpark = new Spark (0);
    }

    public Command setRainbow (){
        return Commands.runOnce(()->ledSpark.set(-0.97), this);
    }
    
    public Command INcargo(){
        return Commands.runOnce(()->ledSpark.set(-0.97));
    }
    
    public Command launched(){
        return Commands.runOnce(()->ledSpark.set(-0.97));
    }
    
    public Command enemyspoted(){
        return Commands.runOnce(()->ledSpark.set(-0.97));
    }
}