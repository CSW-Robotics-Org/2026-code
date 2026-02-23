package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    private SparkMax hopper_motor;
    // hopper reletive encoder
    private RelativeEncoder hopper_encoder;
    // stores the target rpm
    public double hopperTargetRPM = 0;
    

    // creates a pid controller for the rollers
    private PIDController hopperPID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the roller
    private SimpleMotorFeedforward hopperFF = new SimpleMotorFeedforward(0.2, 0.0021);

    /**
     * Constructor
     * @param r_id (id of the motor that moves the rollers)
     */
    public Hopper(int r_id){
        hopper_motor = new SparkMax(r_id, MotorType.kBrushless);
        hopper_encoder = hopper_motor.getEncoder();

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        hopper_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * sets the speed of the hopper motor
     * @param speed
     */
    public void setHopperMotor(double speed){
        speed = MathUtil.clamp(speed, -1, 1);
        hopperTargetRPM = speed*5676;
    }

    // runs every 20 ms
    public void periodic(){
        // hopper loop to hold speed
        double hopperCurrentRPM = hopper_encoder.getVelocity();
        double hopperPIDOutput = hopperPID.calculate(hopperCurrentRPM,hopperTargetRPM);
        double hopperffOutput = hopperFF.calculate(hopperTargetRPM);
        double hopperVoltage = hopperPIDOutput + hopperffOutput;
        hopperVoltage = MathUtil.clamp(hopperVoltage, -12, 12);
        hopper_motor.setVoltage(hopperVoltage);

    }


}
