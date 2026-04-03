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

    // creates the motor that will rotate the intake downwards
    public SparkMax left_rot_motor;
    // creates the motor that will rotate the intake downwards
    public SparkMax right_rot_motor;
    // creates a variable to store the position of the intake arm
    private double intakePos = 0;
    // stores the encoder of the rotation motor so that we can measure the rotation
    private RelativeEncoder leftRotEncoder;

    // pid controller to hold rotation
    private double rotationTargetDegrees = 90;  // 0 = down, 90 = up
    private PIDController rotationPID = new PIDController(0.01, 0, 0.0);

    // creates a pid controller for the feeder
    private PIDController intakePID = new PIDController(0.0004, 0, 0);
    // creates the feed forward for the feeder
    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0.2, 0.0021);

    /**
     * Constructor
     * @param intake_id (id of the intake motor)
     * @param left_rot_id (id of the left rotation motor)
     * @param right_rot_id (id of the right rotation motor)
     */
    public Intake(int i_id, int left_rot_id,int right_rot_id){
        intake_motor = new SparkMax(i_id, MotorType.kBrushless);
        intake_encoder  = intake_motor.getEncoder();
        left_rot_motor = new SparkMax(left_rot_id, MotorType.kBrushless);
        right_rot_motor = new SparkMax(right_rot_id, MotorType.kBrushless);
        leftRotEncoder = left_rot_motor.getEncoder();

        // creates a new config for the motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // applys the config to the motors
        intake_motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // tell the encoder we start facing straight up
        double startingRotations = 90.0 / 360.0; // convert 90° to rotations (encoder units)
        leftRotEncoder.setPosition(startingRotations);

    }

    /** 
     * Sets the intake arm to a target angle (degrees)
     * @param degrees 0 = down, 90 = up
     */
    public void setRotationTarget(double degrees){
        rotationTargetDegrees = MathUtil.clamp(degrees, 0, 180);
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
        // // updates the intake possition
        // intakePos = leftRotEncoder.getPosition();
        // double currentDegrees = intakePos * 360; // convert rotations to degrees

        // System.out.println("Target Degrees: " + rotationTargetDegrees);
        // System.out.println("Current Degrees: " + currentDegrees);

        // // PID loop to move the arm to the target
        // double pidOutput = rotationPID.calculate(currentDegrees, rotationTargetDegrees);
        // pidOutput = MathUtil.clamp(pidOutput, -1, 1);
        // System.out.println("PID output: " + pidOutput);

        // left_rot_motor.set(pidOutput);
        // right_rot_motor.set(-pidOutput);

        // intake loop to hold speed
        double intakeCurrentRPM = intake_encoder.getVelocity();
        double intakePIDOutput = intakePID.calculate(intakeCurrentRPM,intakeTargetRPM);
        double intakeffOutput = intakeFF.calculate(intakeTargetRPM);
        double intakeVoltage = intakePIDOutput + intakeffOutput;
        intakeVoltage = MathUtil.clamp(intakeVoltage, -12, 12);
        intake_motor.setVoltage(intakeVoltage);
    }




}
