package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
  //creates shooter motor
    private SparkMax s_motor;
    //stores max speed of the s_motor
    public double sMaxSpeed=1;
    private SparkMax rot_motor;
   //creates turret rotation motor
  public double rMaxSpeed=1;
  //the constructor that helps create the motor

  //store current rotation
  private double turretRotation = 0;
    // this is our ficticous current rotation
    private double currentRot = 0;

   public Turret (int m_id1, int m_id2){
        s_motor = new SparkMax(m_id1, MotorType.kBrushless);
        rot_motor = new SparkMax (m_id2,MotorType.kBrushless);
    }
//sets the shooter motor speed
    public void SetShooter(double speed){
        s_motor.set(speed);
Math.min(sMaxSpeed, speed);
    }
//sets turret motor speed
     public void SetTurretMotor(double speed){
        if(turretRotation >=85 && (speed > 0)){  s_motor.set(0);}
     
    }
     s_motor.set(speed);
Math.min(rMaxSpeed, speed);
    //method that runs every 20 milliseconds (aproximite)
    public void periodic(){
        turretRotation = currentRot;

    }
}
