// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
	@NotLogged
	private XboxController driver = new XboxController(DriverConstants.DriverPort);

	private Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-driver.getLeftY(), true)
			* RobotConstants.maxSpeed;
	private Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-driver.getLeftX(), true)
			* RobotConstants.maxSpeed;
	private Supplier<Double> rightX = () -> DriverConstants.joystickDeadband(-driver.getRightX(), true)
			* RobotConstants.maxRot;

	private Swerve swerve = TunerConstants.swerve;

	private Trigger resetGyro = new Trigger(() -> driver.getBButton());

	private Trigger pidtoPose = new Trigger(() -> driver.getAButton());
	private Trigger pointDrive = new Trigger(() -> driver.getXButton());
	private Trigger bumpDrive = new Trigger(() -> driver.getYButton());

	public RobotContainer() {
		configureBindings();
		configureControllerAlerts();
	}

	private void configureBindings() {
		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true));

		resetGyro.onTrue(Commands.runOnce(() -> swerve.resetGyro(), swerve));
		pidtoPose.whileTrue(swerve.pidtoPose(() -> FieldUtil.getTowerCenter()));
		pointDrive.whileTrue(swerve.pointDrive(leftY, leftX, () -> FieldUtil.getHubCenter(), () -> true));
		bumpDrive.whileTrue(
				Commands.run(() -> swerve.angularDriveRequest(leftY, leftX, () -> swerve.getClosest45()), swerve));
	}

	public void configureControllerAlerts() {
		// 10/20sec endgame alert
		new Trigger(() -> {
			return DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0.0
					&& ((DriverStation.getMatchTime() <= Math.round(20)
							&& DriverStation.getMatchTime() > Math.round(13))
							|| DriverStation.getMatchTime() <= Math.round(10));
		})
				.onTrue(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 1.0);
				}).withTimeout(1.5).andThen(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 0.0);
				}).withTimeout(1.0)));

		// 5 sec before hub switch alert
		new Trigger(() -> {
			return DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0.0
					&& FieldUtil.getShiftTimeLeft() <= Math.round(5);
		})
				.onTrue(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 1.0);
				}).withTimeout(1.5).andThen(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 0.0);
				}).withTimeout(1.0)));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
