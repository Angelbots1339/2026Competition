// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriverConstants;

@Logged
public class RobotContainer {
	@NotLogged
	private XboxController driver = new XboxController(DriverConstants.DriverPort);

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
