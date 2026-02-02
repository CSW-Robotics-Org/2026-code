package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
public class Intake extends SubsystemBase {
    //Slurp motor
    //Lower the arm motor
    //Max speed (Slurp motor)
    //Max speed (Lower the arm motor)
    //Current angle of rotation
    //Wanted angle of rotation
    //Max angle of rotation
    //
    //METHODS
    //Periodic();-> runs every 20 mili seconds
    //Constructor -> create motors
    //Intake-> sets slurp motor
    //Extend-> sets arm motor

    SparkMax SlurpMotor;
    SparkMax LowerTheArmMotor;

    public Intake (int s1_id, int s2_id){
        SlurpMotor = new SparkMax(s1_id, MotorType.kBrushless);
    }
    private double intakeRotation = 0;
    private double wantedIntakeRotation = 0;
    public void periodic(){
        intakeRotation = wantedIntakeRotation;
    }

}
