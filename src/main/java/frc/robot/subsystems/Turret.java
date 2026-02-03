package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    
    // Creates the shooter motor
    private SparkMax s1_motor;
    // Stores the max speed of the s_motor
    public double s1MaxSpeed = 1;

    // Creates the turret rotation motor
    private SparkMax rot_motor;
    // Stores the max speed of the rot_motor
    public double rMaxSpeed = 1;

    // creates the intake motor
    private SparkMax feed_motor;
    // intake max speed
    public double fMaxSpeed = 1;

    // Variable to store the current rotation
    private double turretRotation = 0;
    // this is our ficticous current rotation
    private double currentRot = 0;

    // creates the limit switches
    private DigitalInput left_lim_switch = new DigitalInput(0);
    private DigitalInput right_lim_switch = new DigitalInput(1);

    // The constructor that creates the motors
    public Turret(int s1_id, int rot_id, int feed_id){
        s1_motor = new SparkMax(s1_id, MotorType.kBrushless);
        rot_motor = new SparkMax(rot_id, MotorType.kBrushless);
        feed_motor = new SparkMax(feed_id, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(1).idleMode(IdleMode.kBrake);

        s1_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rot_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feed_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    // Sets the shooter motor speed
    public void SetShooterMotor(double speed){
        s1_motor.set(
            Math.min(s1MaxSpeed, speed)
        );
    }

    // Sets the turret motor speed
    public void SetTurretMotor(double speed){
        // if we are all the way to the right and we tell the turret to go right
        if (((turretRotation >= 85) || (right_lim_switch.get() == true)) && (speed > 0)){
            // freeze the rotational motor
            rot_motor.set(0);
        }
         // if we are all the way to the left and we tell the turret to go left
        else if (((turretRotation <= -85) || (left_lim_switch.get() == true)) && (speed < 0)){
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
    public void SetFeederMotor(double speed){
        feed_motor.set(
            Math.min(fMaxSpeed, speed)
        );
    }

     // Method that runs ~ every 20 ms
    public void periodic(){
        turretRotation = currentRot;
    }


    
}
