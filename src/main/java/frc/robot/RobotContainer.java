// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
	@NotLogged
	private XboxController driver = new XboxController(DriverConstants.DriverPort);

	private Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-driver.getLeftY(), true) * RobotConstants.maxSpeed;
	private Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-driver.getLeftX(), true) * RobotConstants.maxSpeed;
	private Supplier<Double> rightX = () -> DriverConstants.joystickDeadband(-driver.getRightX(), true) * RobotConstants.maxRot;
	
	private Swerve swerve = TunerConstants.swerve;
	
	private Trigger resetGyro = new Trigger(() -> driver.getBButton());

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true));
		
		resetGyro.onTrue(Commands.runOnce(() -> swerve.resetGyro(), swerve));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
