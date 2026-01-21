package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Hopper {
    // Vars:
    /*
     * Motor 1, motor2, maxspeed
     */
    // Methods;
    /*
     * Constructor
     * Set motors
     */
    
    private SparkMax motor_1;
    private SparkMax motor_2;
    public double mMaxSpeed = 1;
   
    public Hopper (int m1_id, int m2_id) {
        motor_1 = new SparkMax(m1_id, MotorType.kBrushless);
        motor_2 = new SparkMax(m2_id, MotorType.kBrushless);
    }

    // Command for Shooter Speed
    public void MotorShoot() {
        
    }

}
