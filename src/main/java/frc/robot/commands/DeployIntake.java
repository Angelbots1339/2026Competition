// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployIntake extends Command {
	Intake m_Intake = new Intake();
	Angle m_goalAngle;

	/** Creates a new DeployIntake. */
	public DeployIntake(Intake intake, Angle angle) {
		m_Intake = intake;
		m_goalAngle = angle;
		addRequirements(intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_Intake.setIntakeAngle(m_goalAngle);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
