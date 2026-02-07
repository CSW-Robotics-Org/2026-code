package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.geometry.Translation3d;



public class TurretTracking extends Command {

    public static Command lineUpHub(Turret turret, LimeLight limelight){
        // the tag position
        Pose3d tagPos = new Pose3d(
            limelight.targetPoseRobotSpace[0],
            limelight.targetPoseRobotSpace[1],
            limelight.targetPoseRobotSpace[2],
            new Rotation3d(
                Math.toRadians(limelight.targetPoseRobotSpace[3]),
                Math.toRadians(limelight.targetPoseRobotSpace[4]),
                Math.toRadians(limelight.targetPoseRobotSpace[5]),
            )
        );

        // translation of the tag to the center of the box
        Transform3d tagToBoxCenter = new Transform3d(
            new Translation3d(
                -0.5969,
                0,
                0
            ),
            new Rotation3d()   
        );

        // transforms the tag position by the tag to box center transformation
        Pose3d boxCenterPos = tagPos.transformBy(tagToBoxCenter);

        // the angle offset we want
        double targetAngle = Math.toDegrees(
            Math.atan2(
                boxCenterPos.getX(),
                boxCenterPos.getY()
            )
        );


        PIDController turretPID = new PIDController(
            0.02,   // kP
            0.0,    // kI
            0.00   // kD
        );

        turretPID.enableContinuousInput(-180.0, 180.0);
        turretPID.setTolerance(0.75); // degrees

        // calculates the speed we need
        double turretSpeed = turretPID.calculate(
            targetAngle, 
            0.0             
        );

        // clamps the speed
        turretSpeed = MathUtil.clamp(turretSpeed, -0.35, 0.35);

        turret.setTurretMotor(turretSpeed);
       

    }

}
