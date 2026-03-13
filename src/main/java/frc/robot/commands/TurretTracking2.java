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

    public double tagError;

    private final Turret turret;
    private final LimeLight limelight;
    
    /**
     * Constructor of the command. Sets the turret target angle based on pos data.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     */
    public TurretTracking2(Turret turret, LimeLight limelight) {
        this.turret = turret;
        this.limelight = limelight;
    } 

    public void execute() {

        // if we cant see a tag dont move
        if (!limelight.hasTarget()){
            turret.setRotationMotor(0);
        }

        //double tx = LimelightHelpers.getTX("limelight");

        double tagError = LimelightHelpers.getTY("limelight-front");
        // double targetX = limelight.robotPosTargetSpace[0];
        // double targetZ = limelight.robotPosTargetSpace[1];
        double PIDoutput = turretPID.calculate(tagError, 0);
        
        System.out.println("tag error: " + tagError);
        System.out.println("PIDoutput: " + PIDoutput);
        turret.setRotationMotor(PIDoutput);
    }

    public boolean rotationReady(){
        return Math.abs(tagError) < 5;
    }

    public void end(boolean interrupted) {
        turret.setRotationMotor(0);
    }

    public boolean isFinished() {
        return false; // never ends on its own
    }

}
