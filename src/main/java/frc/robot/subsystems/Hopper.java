package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hopper {


    // make motors visible to class
    SparkMax top_motor;
    SparkMax bot_motor;

    // set max speed
    double topMaxSpeed = 1;
    double botMaxSpeed = 1;

    // make motor
    public Hopper(int t_id, int b_id){
        top_motor = new SparkMax(t_id, MotorType.kBrushless);
        bot_motor = new SparkMax(b_id, MotorType.kBrushless);
    }

    // sets the motors which makes the motors set the motors to set the motors which to sets the motors to set the motors to set the motors to set the speed of the motors
    public void SetMotors(double sp1, double sp2){
        top_motor.set(Math.min(sp1, topMaxSpeed));
        bot_motor.set(Math.min(sp2, botMaxSpeed));
    }
}
