package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
  //creates shooter motor
    private SparkMax s_motor;
 
      private SparkMax feeder_motor;
    //stores max speed of the s_motor
    public double sMaxSpeed=1;

    public double fMaxSpeed=1;
    private SparkMax rot_motor;
   //creates turret rotation motor
  public double rMaxSpeed=1;
  //the constructor that helps create the motor

  //store current rotation
  private double turretRotation = 0;
    // this is our ficticous current rotation
    private double currentRot = 0;


    private DigitalInput left_lim_switch = new DigitalInput (0);
    private DigitalInput right_lim_switch = new DigitalInput (1);

   public Turret (int m_id1, int m_id3, int m_id4){
        s_motor = new SparkMax(m_id1, MotorType.kBrushless);
        rot_motor = new SparkMax (m_id3,MotorType.kBrushless);
    feeder_motor= new SparkMax (m_id4,MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(1).idleMode(IdleMode.kBrake);

    s_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rot_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feeder_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
//sets the shooter motor speed
    public void SetShooter(double speed){
        s_motor.set(speed);
Math.min(sMaxSpeed, speed);
    }

        public void SetFeeder(double speed){
        feeder_motor.set(speed);
Math.min(fMaxSpeed, speed);
    }
//sets turret motor speed
     public void SetTurretMotor(double speed){
        if((turretRotation >=85) || (right_lim_switch.get() == true) && (speed > 0)){
            s_motor.set(0);
        }
        if(turretRotation >=-85 || (left_lim_switch.get() == true) && (speed < 0)){  
            s_motor.set(0);
        }
        else {
          s_motor.set(speed);  
        }
     s_motor.set(speed);
            Math.min(rMaxSpeed, speed);

     }
    //method that runs every 20 milliseconds (aproximite)
    public void periodic(){
        turretRotation = currentRot;

    }


}
