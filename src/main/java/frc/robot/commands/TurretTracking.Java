package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;


public class TurretTracking extends Command {

    public static double lineUpHub(LimeLight limelight){
        // the tag position
        Pos3d tagPos = new Pos3d(
            limelight.targetpose_robotspace[0],
            limelight.targetpose_robotspace[1],
            limelight.targetpose_robotspace[2],
            new Rotation3d(
                Math.toRadians(limelight.targetpose_robotspace[3]),
                Math.toRadians(limelight.targetpose_robotspace[4]),
                Math.toRadians(limelight.targetpose_robotspace[5]),
            )
        );

        // translation of the tag to the center of the box
        Transform3d tagToBoxCenter = new Transform3d(
            new Translation3d(
                -0.5969,
                0,
                0,
                new Rotation3d()
            )
        );

        // transforms the tag position by the tag to box center transformation
        Pose3d boxCenterPos = tagPos.transformBy(tagToBoxCenter);

        // the angle offset we want
        double yawOffset = Math.toDegrees(Math.atan2(boxCenterPos.getX(),boxCenterPos.getY());)




    }

}
