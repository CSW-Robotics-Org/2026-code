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
    
    // Hub position (meters)
    private static final double HUB_X = 8.27;   // lateral
    private static final double HUB_Z = 4.03;   // forward

    // creates a pid for the turret
    private static final PIDController turretPID = new PIDController(0.004, 0, 0.000001);

    /**
     * returns the speeed of the turret to point at the center of the hub
     * @param limelight
     * @param turret
     * @return speed (percent %)
     */
    public static void TurretLineup(LimeLight limelight,Turret turret) {

        Translation2d targetPos;

        // if we cant see a tag dont move
        if (limelight.tv == 0){
            turret.setRotationMotor(0);
        }

        //double tx = LimelightHelpers.getTX("limelight");

        double ty = limelight.ty;
        // double targetX = limelight.robotPosTargetSpace[0];
        // double targetZ = limelight.robotPosTargetSpace[1];
        double PIDoutput = turretPID.calculate(ty, 0);
        
        System.out.println("ty: " + ty);
        System.out.println("PIDoutput: " + PIDoutput);
        turret.setRotationMotor(PIDoutput);
        



        // // Use Limelight
        // Pose3d tagPos = new Pose3d(
        //     limelight.robotPosTargetSpace[0],
        //     limelight.robotPosTargetSpace[1],
        //     limelight.robotPosTargetSpace[2],
        //     new Rotation3d(
        //         Math.toRadians(limelight.robotPosTargetSpace[3]),
        //         Math.toRadians(limelight.robotPosTargetSpace[4]),
        //         Math.toRadians(limelight.robotPosTargetSpace[5])
        //     )
        // );

}

    /**
     * Returns the shooter power for the distance to the hub
     * @param limelight
     * @param drivetrain
     * @param turret
     * @return shooter power (double)
     */
    public static double ShooterPower(LimeLight limelight, CommandSwerveDrivetrain drivetrain,Turret turret) {

        Translation3d targetPos;
        double targetDistance = 0;

        if (limelight.tv == 1) {
            Pose3d tagPos = new Pose3d(
                limelight.targetPoseRobotSpace[0],
                limelight.targetPoseRobotSpace[1],
                limelight.targetPoseRobotSpace[2],
                new Rotation3d(
                    Math.toRadians(limelight.targetPoseRobotSpace[3]),
                    Math.toRadians(limelight.targetPoseRobotSpace[4]),
                    Math.toRadians(limelight.targetPoseRobotSpace[5])
                )
            );

            Transform3d tagToBoxCenter = new Transform3d(
                new Translation3d(-0.5969, 0, 0),
                new Rotation3d()
            );

            Pose3d boxCenterPos = tagPos.transformBy(tagToBoxCenter);
            targetPos = boxCenterPos.getTranslation();
            // stores the target distance
            targetDistance = Math.hypot(targetPos.getX(), targetPos.getZ());

        } else {
            // rotate relative vector to hub
            double xRel = HUB_X - drivetrain.getState().Pose.getX();
            double zRel = HUB_Z - drivetrain.getState().Pose.getY();

            // stores the target distance
            targetDistance = Math.hypot(xRel, zRel);
        }

        // Quadratic formula for shooter power
        double shooterPow = 0.5 + 0.0199 * targetDistance + 0.00347 * Math.pow(targetDistance, 2);
        shooterPow = MathUtil.clamp(shooterPow,-1,1);

        return shooterPow;
    }


}
