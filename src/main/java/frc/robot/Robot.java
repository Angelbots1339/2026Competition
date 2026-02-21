// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.commands.PathfindingCommand;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldUtil;
import frc.lib.util.tuning.TuningManager;

@Logged(importance = Importance.CRITICAL)
public class Robot extends TimedRobot {
	private Command m_autonomousCommand = Commands.none();

	private final RobotContainer m_robotContainer;

	public Robot() {
		m_robotContainer = new RobotContainer();
		PathfindingCommand.warmupCommand().schedule();
		// doglog log thread takes some time to start up so warm it up
		DogLog.log("", "");

		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		Epilogue.configure(config -> {
			config.loggingPeriod = Seconds.of(0.01);
			// config.minimumImportance = Importance.CRITICAL;
			config.minimumImportance = Importance.DEBUG;
		});
		Epilogue.bind(this);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		FieldUtil.getShiftOrder();
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		m_robotContainer.testingInit();
	}

	@Override
	public void testPeriodic() {
		TuningManager.changeMode();
	}

	@Override
	public void testExit() {
	}
}
