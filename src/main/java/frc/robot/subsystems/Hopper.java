package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{
    
    // creates a motor for the rollers in the hopper
    private TalonFX hopper_motor;
    // // stores the target rpm
    // public double hopperTargetRPM = 0;
    

    // // creates a pid controller for the rollers
    // private PIDController hopperPID = new PIDController(0.0004, 0, 0);
    // // creates the feed forward for the roller
    // private SimpleMotorFeedforward hopperFF = new SimpleMotorFeedforward(0.2, 0.0021);

    /**
     * Constructor
     * @param r_id (id of the motor that moves the rollers)
     */
    public Hopper(int r_id){
        hopper_motor = new TalonFX(r_id);
    }

    /**
     * sets the speed of the hopper motor
     * @param speed
     */
    public void setHopperMotor(double speed){
        hopper_motor.set(-speed);
    }

    // runs every 20 ms
    public void periodic(){

    }


}
