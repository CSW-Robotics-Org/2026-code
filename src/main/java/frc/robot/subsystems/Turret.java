package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    
    // Creates the shooter motor
    private SparkMax s_motor;
    // Stores the max speed of the s_motor
    public double s1MaxSpeed = 1;

    // Creates the turret rotation motor
    private SparkMax rot_motor;
    // Stores the max speed of the rot_motor
    public double rMaxSpeed = 1;
    // Variable to store the current rotation
    private double turretRotation = 0;
    // this is our ficticous current rotation
    private double currentRot = 0;
    // stores the encoder for the sparkmax
    private RelativeEncoder rot_encoder;

    // creates the intake motor
    private SparkMax feed_motor;
    // intake max speed
    public double fMaxSpeed = 1;

    // creates the limit switches
    private DigitalInput left_lim_switch = new DigitalInput(0);
    private DigitalInput right_lim_switch = new DigitalInput(1);

    // The constructor that creates the motors
    public Turret(int s1_id,int s2_id, int rot_id, int feed_id){
        s_motor = new SparkMax(s1_id, MotorType.kBrushless);
        rot_motor = new SparkMax(rot_id, MotorType.kBrushless);
        rot_encoder = rot_motor.getAlternateEncoder();
        feed_motor = new SparkMax(feed_id, MotorType.kBrushless);

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        s_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rot_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feed_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets the shooter motor speed
    public void setShooterMotor(double speed){
        s_motor.set(
            Math.min(s1MaxSpeed, speed)
        );
    }

    // Sets the turret motor speed
    public void setTurretMotor(double speed){
        // if we are all the way to the right and we tell the turret to go right
        if (((turretRotation >= 85) || right_lim_switch.get() == true) && (speed > 0)){
            // freeze the rotational motor
            rot_motor.set(0);
        }
         // if we are all the way to the left and we tell the turret to go left
        else if (((turretRotation <= -85) || left_lim_switch.get() == true) && (speed < 0)){
            // freeze the rotational motor
            rot_motor.set(0);
        }
        // only if both of those are not true do we set the motor
        else{
            rot_motor.set(
                    Math.min(rMaxSpeed, speed)
            );
        }

        }

    // Sets the shooter motor speed
    public void setFeederMotor(double speed){
        feed_motor.set(
            Math.min(fMaxSpeed, speed)
        );
    }

     // Method that runs ~ every 20 ms
    public void periodic(){
        turretRotation = currentRot;

        // some logic to make sure the robot doesnt overturn the turret.
        if (((turretRotation <= -85) || left_lim_switch.get() == true) && ( rot_encoder.getVelocity() > 0)){
            rot_motor.set(0);
        }
        if (((turretRotation >= 85) || right_lim_switch.get() == true) && ( rot_encoder.getVelocity() < 0)){
            rot_motor.set(0);
        }

    }


    
}
