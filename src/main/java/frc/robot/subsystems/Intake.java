package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    // creates a motor for the intake
    private SparkMax i_motor;
    // creates a max speed variable
    public double iMaxSpeed = 1;

    // creates the motor that will rotate the intake downwards
    private SparkMax rot_motor;
    // creates a max speed variable
    public double rotMaxSpeed = 1;
    // creates a variable to store the position of the intake arm
    private double intakePos = 90;
    // this is our ficticous current rotation
    private double currentRot = 0;

    // constructor creates the motors
    public Intake(int i_id, int rot_id){
        i_motor = new SparkMax(i_id, MotorType.kBrushless);
        rot_motor = new SparkMax(rot_id, MotorType.kBrushless);
    }

    // a method to set the motor speed for the intake
    public void setIntakeMotor(double speed){
        i_motor.set(
            Math.min(iMaxSpeed, speed)
        );
    }

    // a method to set the motor speed for the rotation motor
    public void setRotationMotor(double speed){
        rot_motor.set(
            Math.min(rotMaxSpeed, speed)
        );
    }

    // Method that runs ~ every 20 ms
    public void periodic(){
        intakePos = currentRot;
    }




}
