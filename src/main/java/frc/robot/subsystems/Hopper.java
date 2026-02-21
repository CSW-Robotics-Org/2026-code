package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{
    
    // creates a motor for the rollers in the hopper
    private SparkMax roller_motor;
    // creates a max speed variable
    public double rMaxSpeed = 1;

    // creates a motor for the prefeeding mechanum wheels
    private SparkMax prefeed_motor;
    // creates a max speed variable
    public double pMaxSpeed = 1;

    // constructor that creates the motors
    public Hopper(int r_id, int pf_id){
        roller_motor = new SparkMax(r_id, MotorType.kBrushless);
        prefeed_motor = new SparkMax(pf_id, MotorType.kBrushless);

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        roller_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        prefeed_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // a method to set the motor speed for the rollers
    public void setRollerMotor(double speed){
        roller_motor.set(
            Math.min(rMaxSpeed, speed)
        );
    }

    // a method to set the motor speed for the rollers
    public void setPreFeederMotor(double speed){
        prefeed_motor.set(
            Math.min(pMaxSpeed, speed)
        );
    }


}
