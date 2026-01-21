package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake {
    
    // motor
    SparkMax m_motor;
    SparkMax ex_motor;

    // max speeds
    double mMaxSpeed = 1;
    double exMaxSpeed = 1;

    // angles

    // the current angle
    double nowAngle = 0;

    // the needed angle
    double plsAngle;

    // the max angle
    double maxAngle = 180;

    // Constructs motors
    public Intake(int m_id, int ex_id){
        m_motor = new SparkMax(m_id, MotorType.kBrushless);
        ex_motor = new SparkMax(ex_id, MotorType.kBrushless);
    }

    public void StartIntake(double mSpeed){
        // sets the main intake motor speed
        m_motor.set(Math.min(mMaxSpeed, mSpeed));
    }

    public void ExtendIntake(double angle){
        // sets the angle needed, motor speed handled in periodic
        plsAngle = angle;
    }

    public void robotPeriodic(){
        // sets max angle
        if (nowAngle >= 180 || nowAngle <= 180){
            ex_motor.set(0);
        } else {
            // makes lower arm thing motor do it's thing

            // makes a range of 2 degrees below, so it doesn't have to be 100% precise
            if (plsAngle > nowAngle-2){
                // sets motor speed proportional to the difference of the angles
                ex_motor.set(Math.min(-maxAngle / (Math.abs(plsAngle - nowAngle)), exMaxSpeed));

            // makes a range of 2 degrees above, so it doesn't have to be 100% precise
            } else if (plsAngle < nowAngle+2) {
                // sets motor speed proportional to the difference of the angles
                ex_motor.set(Math.min(maxAngle / (Math.abs(plsAngle - nowAngle)), exMaxSpeed));
            }
        }

        // would enter code to set the angle here, don't know how yet
    }

}
