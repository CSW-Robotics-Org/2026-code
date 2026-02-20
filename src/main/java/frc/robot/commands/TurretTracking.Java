package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Turret;

public class TurretTracking extends Command{

    private static final PIDController turretPID = new PIDController(0.02, 0, 0.0);

    static {
        turretPID.enableContinuousInput(-180.0, 180.0);
        turretPID.setTolerance(0.75);
    }

    // Hub position (meters)
    private static final double HUB_X = 8.27;   // lateral
    private static final double HUB_Z = 4.03;   // forward

    /** Returns turret speed */
    public static double TurretLineup(LimeLight limelight, CommandSwerveDrivetrain drivetrain,Turret turret) {

        Translation3d targetPos;
        double targetAngle = 0;

        if (limelight.tv == 1) {
            // Use Limelight
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

            double turretSpeed = turretPID.calculate(targetAngle,turret.getAngle());
            turretSpeed = MathUtil.clamp(turretSpeed, -0.35, 0.35);

            return turretSpeed;

        } 
        else {
            return turret.rotCurrentSpeed;
        }
    }

    /** Returns shooter power using Limelight or robot pose */
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
        double shooterPow = 0.453 + 0.0199 * targetDistance + 0.00347 * Math.pow(targetDistance, 2);

        return shooterPow;
    }
}
