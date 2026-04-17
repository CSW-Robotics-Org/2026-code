package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    // creates a motor for the intake
    private SparkMax intake_motor;
    // creates a max speed variable
    public double iMaxSpeed = 1;
    // intake reletive encoder
    private RelativeEncoder intake_encoder;
    // stores the target rpm
    public double intakeTargetRPM = 0;

    // creates a pid controller for the feeder
    private PIDController intakePID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the feeder
    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0.2, 0.0021);

    /**
     * Constructor
     * @param intake_id (id of the intake motor)
     */
    public Intake(int i_id){
        intake_motor = new SparkMax(i_id, MotorType.kBrushless);
        intake_encoder  = intake_motor.getEncoder();

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        intake_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Sets the speed of the intake motor
     * @param speed
     */
    public void setIntakeMotor(double speed){
        speed = MathUtil.clamp(speed,-1,1 );
        intakeTargetRPM = speed*5676;
        System.out.println("INtake working?");
        System.out.println(intakeTargetRPM);
    }

    // Method that runs ~ every 20 ms
    public void periodic(){

        // intake loop to hold speed
        double intakeCurrentRPM = intake_encoder.getVelocity();
        double intakePIDOutput = intakePID.calculate(intakeCurrentRPM,intakeTargetRPM);
        double intakeffOutput = intakeFF.calculate(intakeTargetRPM);
        double intakeVoltage = intakePIDOutput + intakeffOutput;
        intakeVoltage = MathUtil.clamp(intakeVoltage, -12, 12);
        intake_motor.setVoltage(intakeVoltage);
    }




}
