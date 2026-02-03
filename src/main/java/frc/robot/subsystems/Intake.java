package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    private SparkMax slurp_motor;
    private SparkMax LtA_motor;
    public double slurp_MaxSpeed = 1; 
    public double LtA_MaxSpeed = 1;
    public double curr_AoR = 0;
    public double max_AoR = 360;
    public double wanted_AoR; // don't set this to a number

    public Intake(int slurp_id, int LtA_id) {
        slurp_motor = new SparkMax(slurp_id, MotorType.kBrushless);
        LtA_motor = new SparkMax(LtA_id, MotorType.kBrushless);
    }

    public void Set_Slurp(double speed) {
        slurp_motor.set(
            Math.min(slurp_MaxSpeed,speed)
        );
    }

    public void Set_LtA(double speed) {
        LtA_motor.set(
            Math.min(LtA_MaxSpeed,speed)
        ); 
    }

    // have a periodic method that sets the curr aor to the wanted aor
    public void periodic() {
        curr_AoR = wanted_AoR;
    }
}