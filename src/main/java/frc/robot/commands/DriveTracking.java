
package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;


public class DriveTracking extends Command {

    /**
     * A command to lineup close tracking like last year. Assumes limelight is on front of robot
     * @param drivetrain (drivetrain to drive)
     * @param limelight (limelight on the front of robot)
     * @return SwerveRequest.RobotCentric()
     */
    public static SwerveRequest lineUp(CommandSwerveDrivetrain drivetrain, LimeLight limelight) {
  
        return new SwerveRequest.RobotCentric() // Robot-centric mode
            .withVelocityX(
                Math.copySign(
                    Math.min(
                        Math.abs((limelight.targetPosCameraSpace[2]-limelight.zoffset))*10,
                        1), // the speed that we limit the limelights at
                    limelight.targetPosCameraSpace[2]-limelight.zoffset
                )
            )   

            .withVelocityY(
                Math.copySign(
                    Math.min(
                        Math.abs((limelight.targetPosCameraSpace[0]-limelight.xoffset)),
                        0.8), // the speed that we limit the limelights at
                    -limelight.targetPosCameraSpace[0]-limelight.xoffset
                )
            )   
            .withRotationalRate(
                Math.copySign(
                    Math.min(
                        Math.abs((limelight.targetPosCameraSpace[4]-limelight.rotoffset)),
                        0.3), // the speed that we limit the limelights at
                    -limelight.targetPosCameraSpace[4]-limelight.rotoffset
                )
            );
    }

    /**
     * A command to lineup close tracking like last year. Assumes limelight is on left of robot
     * @param drivetrain (drivetrain to drive)
     * @param limelight (limelight on the front of robot)
     * @return SwerveRequest.RobotCentric()
     */
    public static SwerveRequest lineUpLeft(CommandSwerveDrivetrain drivetrain, LimeLight limelight) {
  
        return new SwerveRequest.RobotCentric() // Robot-centric mode
            .withVelocityX(
                Math.copySign(
                    Math.min(
                        Math.abs((limelight.targetPosCameraSpace[0]-limelight.xoffset)),
                        0.8), // the speed that we limit the limelights at
                    limelight.targetPosCameraSpace[0]-limelight.xoffset
                )
            )   

            .withVelocityY(
                Math.copySign(
                    Math.min(
                        Math.abs((limelight.targetPosCameraSpace[2]-limelight.zoffset))*10,
                        1), // the speed that we limit the limelights at
                    limelight.targetPosCameraSpace[2]-limelight.zoffset
                )
            )   
            .withRotationalRate(
                Math.copySign(
                    Math.min(
                        Math.abs((limelight.targetPosCameraSpace[4]-limelight.rotoffset)),
                        0.3), // the speed that we limit the limelights at
                    -limelight.targetPosCameraSpace[4]-limelight.rotoffset
                )
            );
    }
}
