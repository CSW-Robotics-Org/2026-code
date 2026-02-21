package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    // creates a motor for the intake
    private SparkMax intake_motor;
    // creates a max speed variable
    public double iMaxSpeed = 1;

    // creates the motor that will rotate the intake downwards
    private SparkMax left_rot_motor;
    // creates the motor that will rotate the intake downwards
    private SparkMax right_rot_motor;
    // creates a max speed variable
    public double rotMaxSpeed = 1;
    // creates a variable to store the position of the intake arm
    private double intakePos = 0;
    // stores the encoder of the rotation motor so that we can measure the rotation
    private RelativeEncoder leftRotEncoder;

    // constructor creates the motors
    public Intake(int i_id, int left_rot_id,int right_rot_id){
        intake_motor = new SparkMax(i_id, MotorType.kBrushless);
        left_rot_motor = new SparkMax(left_rot_id, MotorType.kBrushless);
        right_rot_motor = new SparkMax(right_rot_id, MotorType.kBrushless);
        leftRotEncoder = left_rot_motor.getEncoder();

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        intake_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        left_rot_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right_rot_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    // a method to set the motor speed for the intake
    public void setIntakeMotor(double speed){
        intake_motor.set(
            Math.min(iMaxSpeed, speed)
        );
    }

    // a method to set the motor speed for the rotation motor
    public void setRotationMotor(double speed){
        left_rot_motor.set(
            Math.min(rotMaxSpeed, speed)
        );
        right_rot_motor.set(
            Math.min(rotMaxSpeed, -speed)
        );
    }

    // Method that runs ~ every 20 ms
    public void periodic(){
        intakePos = leftRotEncoder.getPosition();
    }




}
