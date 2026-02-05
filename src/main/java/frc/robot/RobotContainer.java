// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.logging.LogManager;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveTracking;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // creates our controllers
    private final CommandXboxController operator = new CommandXboxController(2);
    private final Joystick l_joystick = new Joystick(0);
    private final Joystick r_joystick = new Joystick(1);
    

    // auto picker for sd
    private final SendableChooser<String> m_auto_chooser = new SendableChooser<>();

    // auto picker for command  /* Path follower */
    private SendableChooser<Command> autoChooser;
    
    // creates our limelights
    public final LimeLight limelight = new LimeLight("limelight-front",0,0,0);


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(r_joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(r_joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-l_joystick.getX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );




        // ##### FILE MANAGER FOR AUTO PICKER ON SD #####

            // get all the files in the pathplanner/autos dir
            File[] files_in_deploy_folder = new File(
            Filesystem.getDeployDirectory(),"pathplanner/autos").listFiles((dir, name) -> name.endsWith(".auto")
            );
            
            // and then add them to a list
            for (File i_file : files_in_deploy_folder) {
            if (i_file.isFile()) {
                m_auto_chooser.addOption( // add option to SmartDashboard
                i_file.getName().substring(0, i_file.getName().lastIndexOf(".")), // removed .auto
                i_file.getName().substring(0, i_file.getName().lastIndexOf("."))
                );
            }
            }
            // put it on SmartDashboard
            m_auto_chooser.setDefaultOption("Default", "Default");
            SmartDashboard.putData("Auto Chooser", m_auto_chooser);
        

        // ##### DRIVER CONTROLS #####

            // Theoretically resets the field reletive possitioning
            new JoystickButton(r_joystick,3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            
            // Theoretically applies the break works great in the sim
            new JoystickButton(r_joystick,5).whileTrue(drivetrain.applyRequest(() -> brake));

            // Limelight tracking button
            new JoystickButton(r_joystick,1).whileTrue(drivetrain.applyRequest(()-> DriveTracking.lineUpLeft(drivetrain,limelight)));

            // robot rel
            new JoystickButton(r_joystick,4).whileTrue(drivetrain.applyRequest(()-> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(-r_joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-r_joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-l_joystick.getX() * MaxAngularRate)
            
            ));

         // ##### OPERATOR CONTROLS #####

         // moves the turret based off of the y axis


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        autoChooser = AutoBuilder.buildAutoChooser("Default");
        return autoChooser.getSelected();
    }   
}

