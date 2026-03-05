package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    
    // Creates the shooter motor
    private SparkMax s_motor;
    // stores the encoder for the sparkmax
    public RelativeEncoder s_encoder;
    // stores the target rpm
    public double targetRPM = 0;

    // Creates the turret rotation motor
    public SparkMax rot_motor;
    // stores the encoder for the sparkmax
    private RelativeEncoder rot_encoder;
    // creates a target angle
    public double rotTargetRPM = 0;

    // creates the intake motor
    private SparkMax feed_motor;
    // stores the encoder for the sparkmax
    private RelativeEncoder f_encoder;
    // stores the target rpm
    public double ftargetRPM = 0;

    // creates the limit switches
    private DigitalInput left_lim_switch = new DigitalInput(0);
    private DigitalInput right_lim_switch = new DigitalInput(1);

    // creates a pid controller for the shooter
    private PIDController shooterPID = new PIDController(0.0015, 0, 0);
    // creates the feed forward for the shooter
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.2, 0.0021);

    // creates a pid controller for the feeder
    private PIDController feederPID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the feeder
    private SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(0.2, 0.0021);

    // creates a pid controller for the feeder
    private PIDController rotPID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the feeder
    private SimpleMotorFeedforward rotFF = new SimpleMotorFeedforward(0.2, 0.0021);
    /**
     * Constructor
     * @param shooter_id (id of the shooter motor)
     * @param rotation_id (id of the rotation motor)
     * @param feeder_id (id of the feeder motor)
     */
    public Turret(int s1_id, int rot_id, int feed_id){
        s_motor = new SparkMax(s1_id, MotorType.kBrushless);
        s_encoder = s_motor.getEncoder();
        rot_motor = new SparkMax(rot_id, MotorType.kBrushless);
        rot_encoder = rot_motor.getEncoder();

        feed_motor = new SparkMax(feed_id, MotorType.kBrushless);
        f_encoder = feed_motor.getEncoder();

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        rot_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feed_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Sets the speed of the rotation motor
     * @param speed
     */
    public void setRotationMotor(double speed) {
        // speed = MathUtil.clamp(speed, -1, 1);
        // rotTargetRPM = speed*5676;
        rot_motor.set(speed);
    }

    /**
     * Sets the speed of the shooter motor
     * @param speed
     */
    public void setShooterMotor(double speed){
        speed = MathUtil.clamp(speed, -1, 1);
        targetRPM = speed*5676;
    }

    /**
     * Sets the speed of the shooter motor
     * @param speed
     */
    public void setFeederMotor(double speed){
       speed = MathUtil.clamp(speed, -1, 1);
         ftargetRPM = speed*5676;
    }

    /**
     * A method to see if the shooter is up to speed
     * @return Returns true if the shooter is at targetSpeed
     */
    public boolean atSpeed(){
        double currentRPM = s_encoder.getVelocity();

        // Don’t say we’re at speed if target is basically zero
        if (Math.abs(targetRPM) < 100) {
            return false;
        }

        double tolerance = 100; // RPM tolerance (tune this)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

     // Method that runs ~ every 20 ms
    public void periodic(){

        // turret logic so we dont break the turret
        if (left_lim_switch.get() && rot_encoder.getVelocity() > 0) {
            rot_motor.set(0);
        }
        if (right_lim_switch.get() && rot_encoder.getVelocity()  < 0) {
            rot_motor.set(0);
        }

        // shooter loop to hold speed
        double shooterCurrentRPM = s_encoder.getVelocity();
        double shooterPIDOutput = shooterPID.calculate(shooterCurrentRPM,targetRPM);
        double shooterffOutput = shooterFF.calculate(targetRPM);
        double shooterVoltage = shooterPIDOutput + shooterffOutput;
        shooterVoltage = MathUtil.clamp(shooterVoltage, -12, 12);
        s_motor.setVoltage(shooterVoltage);


        // feeder loop to hold speed
        double feederCurrentRPM = f_encoder.getVelocity();
        double feederPIDOutput = feederPID.calculate(feederCurrentRPM,ftargetRPM);
        double feederffOutput = feederFF.calculate(ftargetRPM);
        double voltage = feederPIDOutput + feederffOutput;
        voltage = MathUtil.clamp(voltage, -12, 12);
        feed_motor.setVoltage(voltage);

        // // turret loop to hold speed
        // double rotCurrentRPM = rot_encoder.getVelocity();
        // double rotPIDOutput = rotPID.calculate(rotCurrentRPM,rotTargetRPM);
        // double rotffOutput = rotFF.calculate(rotTargetRPM);
        // double rotVoltage = rotPIDOutput + rotffOutput;
        // rotVoltage = MathUtil.clamp(rotVoltage, -12, 12);
        // rot_motor.setVoltage(rotVoltage);

    }
    
}
