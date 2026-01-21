package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake {
    
    SparkMax m_motor;
    SparkMax ex_motor;

    double mMaxSpeed = 1;
    double exMaxSpeed = 1;

    double nowAngle = 0;
    double plsAngle;
    double maxAngle = 180;

    public Intake(int m_id, int ex_id){
        m_motor = new SparkMax(m_id, MotorType.kBrushless);
        ex_motor = new SparkMax(ex_id, MotorType.kBrushless);
    }

    public void StartIntake(double mSpeed){
        m_motor.set(Math.min(mMaxSpeed, mSpeed));
    }

    public void ExtendIntake(double angle){
        plsAngle = angle;
    }

    public void robotPeriodic(){
        if (nowAngle >= 180 || nowAngle <= 180){
            ex_motor.set(0);
        } else {
            if (plsAngle > nowAngle){
                ex_motor.set(Math.min(-maxAngle / (Math.abs(plsAngle - nowAngle)), exMaxSpeed));
            } else {
                ex_motor.set(Math.min(maxAngle / (Math.abs(plsAngle - nowAngle)), exMaxSpeed));
            }
        }
    }

}
