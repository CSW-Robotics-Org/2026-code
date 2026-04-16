// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        // puts the shooter offset on sd
        SmartDashboard.putNumber("Shooter Power Offset", m_robotContainer.ShooterPowerCommand.getPowerOffset());
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Battery Voltage", voltage);

        m_robotContainer.field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
        SmartDashboard.putData("Field", m_robotContainer.field);

        double timeLeft = Timer.getMatchTime();
        SmartDashboard.putNumber("Match Time", timeLeft);

        SmartDashboard.putBoolean("FMS Connected", DriverStation.isFMSAttached());

        SmartDashboard.putBoolean("DS Connected", DriverStation.isDSAttached());
        
        
        String state;

        if (DriverStation.isDisabled()) {
            state = "DISABLED";
        } else if (DriverStation.isAutonomous()) {
            state = "AUTO";
        } else if (DriverStation.isTeleop()) {
            state = "TELEOP";
        } else if (DriverStation.isTest()) {
            state = "TEST";
        } else {
            state = "UNKNOWN";
        }

        SmartDashboard.putString("Robot State", state);

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
