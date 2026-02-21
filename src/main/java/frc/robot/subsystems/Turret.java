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
import frc.robot.commands.TurretTracking;

public class Turret extends SubsystemBase{
    
    // Creates the shooter motor
    private SparkMax s_motor;
    // Stores the max speed of the s_motor
    public double s1MaxSpeed = 1;
    // stores the encoder for the sparkmax
    private RelativeEncoder s_encoder;
    // stores the target rpm
    public double targetRPM = 0;

    // Creates the turret rotation motor
    public SparkMax rot_motor;
    // Stores the max speed of the rot_motor
    public double rMaxSpeed = 1;
    // Variable to store the current rotation
    private double turretRotation = 0;
    // stores the encoder for the sparkmax
    private RelativeEncoder rot_encoder;
    // stores the current speed
    public double rotCurrentSpeed = 0;

    // creates the intake motor
    private SparkMax feed_motor;
    // intake max speed
    public double fMaxSpeed = 1;
    // stores the encoder for the sparkmax
    private RelativeEncoder f_encoder;
    // stores the target rpm
    public double ftargetRPM = 0;

    // creates the limit switches
    private DigitalInput left_lim_switch = new DigitalInput(0);
    private DigitalInput right_lim_switch = new DigitalInput(1);

    // creates a pid controller for the shooter
    private PIDController shooterPID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the shooter
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.2, 0.0021);

    // creates a pid controller for the shooter
    private PIDController feederPID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the shooter
    private SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(0.2, 0.0021);

    private static final double TURRET_GEAR_RATIO = 8.0;

    // The constructor that creates the motors
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

    // Sets the shooter motor speed
    public void setShooterMotor(double speed){
        speed = MathUtil.clamp(speed, -1, 1);
        targetRPM = speed*5676;
    }

    // Sets the turret motor speed
    public void setTurretMotor(double speed){
        // if we are all the way to the right and we tell the turret to go right
        if (((turretRotation >= 85) || right_lim_switch.get() == true) && (speed > 0)){
            // freeze the rotational motor
            rot_motor.set(0);
            rotCurrentSpeed = 0;
        }
         // if we are all the way to the left and we tell the turret to go left
        else if (((turretRotation <= -85) || left_lim_switch.get() == true) && (speed < 0)){
            // freeze the rotational motor
            rot_motor.set(0);
            rotCurrentSpeed = 0;
        }
        // only if both of those are not true do we set the motor
        else{
            rot_motor.set(
                    Math.min(rMaxSpeed, speed)
            );
            rotCurrentSpeed = Math.min(rMaxSpeed, speed);
        }
        }

    // Sets the shooter motor speed
    public void setFeederMotor(double speed){
       speed = MathUtil.clamp(speed, -1, 1);
         ftargetRPM = speed*5676;
    }

     // Method that runs ~ every 20 ms
    public void periodic(){
        turretRotation = this.getAngle();

        if ((turretRotation <= -85 || left_lim_switch.get()) && rotCurrentSpeed < 0) {
            rot_motor.set(0);
        }
        if ((turretRotation >= 85 || right_lim_switch.get()) && rotCurrentSpeed > 0) {
            rot_motor.set(0);
        }

        if (left_lim_switch.get() && turretRotation > -84) {
        rot_encoder.setPosition(-85.0 / 360.0 * TURRET_GEAR_RATIO);
        }
        if (right_lim_switch.get() && turretRotation < 84) {
            rot_encoder.setPosition(85.0 / 360.0 * TURRET_GEAR_RATIO);
        }

        double scurrentRPM = s_encoder.getVelocity();
        double spidOutput = shooterPID.calculate(scurrentRPM,targetRPM);
        double sffOutput = shooterFF.calculate(targetRPM);
        double svoltage = spidOutput + sffOutput;
        svoltage = MathUtil.clamp(svoltage, -12, 12);
        s_motor.setVoltage(svoltage);

        double currentRPM = f_encoder.getVelocity();
        double pidOutput = feederPID.calculate(currentRPM,ftargetRPM);
        double ffOutput = feederFF.calculate(ftargetRPM);
        double voltage = pidOutput + ffOutput;
        voltage = MathUtil.clamp(voltage, -12, 12);
        feed_motor.setVoltage(voltage);

    }

    public double getAngle() {
        double motorRotations = rot_encoder.getPosition();
        double turretRotations = motorRotations / TURRET_GEAR_RATIO;
        double turretDegrees = turretRotations * 360.0;
        return turretDegrees;
    }
    
}
