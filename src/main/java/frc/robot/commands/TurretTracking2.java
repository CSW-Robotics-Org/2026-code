package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Turret;

 

public class TurretTracking2 extends Command{

    // creates a pid for the turret
    private static final PIDController turretPID = new PIDController(0.004, 0, 0.000001);

    /**
     * returns the speeed of the turret to point at the center of the hub
     * @param limelight
     * @param turret
     * @return speed (percent %)
     */
    public static void TurretLineup(LimeLight limelight,Turret turret) {

        // if we cant see a tag dont move
        if (!limelight.hasTarget()){
            turret.setRotationMotor(0);
        }

        //double tx = LimelightHelpers.getTX("limelight");

        double ty = LimelightHelpers.getTY("limelight-front");
        // double targetX = limelight.robotPosTargetSpace[0];
        // double targetZ = limelight.robotPosTargetSpace[1];
        double PIDoutput = turretPID.calculate(ty, 0);
        
        System.out.println("ty: " + ty);
        System.out.println("PIDoutput: " + PIDoutput);
        turret.setRotationMotor(PIDoutput);
        

}

}
