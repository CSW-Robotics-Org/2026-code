package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    //*Variables
    //Motor1 (spark max)
    //Motor2 (spark max)
    //Max speed 1 (motor 1)
    //Max speed 2 (motor 2)

    //METHODS
    //Periodic();-> runs every 20 mili seconds
    //Constructor -> create motors
    //Set motors ();-> set motor speed
    //            ^-<-speed
 
        SparkMax motor1;
        SparkMax motor2;
        
    public Hopper(int s1_id, int s2_id){
    motor1 = new SparkMax(s1_id, MotorType.kBrushless);
    motor2 = new SparkMax (s2_id, MotorType.kBrushless);
    }
    public double sMaxSpeed=1;
    public void SetShooter(double speed){
        motor1.set(speed);
Math.min(sMaxSpeed, speed);
    }
}
