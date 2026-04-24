// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldUtil;
import frc.lib.util.Leds;
import frc.lib.util.tuning.TuningManager;
import frc.robot.Constants.VisionConstants;

// Robot.java contains all the logic that will run on the robot when in 
// different states / startup

// this is an example of a logged class using Epilogue.  we can pass options
// into the annotation, to change aspects such as importance, logging method,
// name, etc.
// in this specific example, we make everything in the robot class log everything
// at the critical level
@Logged(importance = Importance.CRITICAL)
public class Robot extends TimedRobot {
	private Command m_autonomousCommand = Commands.none();
	@SuppressWarnings("unused")
	private final Leds led = Leds.getInstance();

	private final RobotContainer m_robotContainer;

	private static final double lowBatteryVoltage = 12.3;
	private static final double criticallyLowBatteryVoltage = 12;
	private static final double lowBatteryDisabledTime = 1.5;

	private final Timer disabledTimer = new Timer();

	public Robot() {
		// this instanciates our robot through the robotcontainer abstraction
		// which contains all of our subsystems, commands, autos, and controller
		// bindings
		m_robotContainer = new RobotContainer();
		// PathfindingCommand.warmupCommand().schedule();
		// dog log is the declarative logging library to quickly log intermediate
		// values/data in functions, methods, commands, etc
		// we can set options such as whether or not to publish the data through NT
		// (by default it does unless it is connected to FMS8, whether to log pdh/canbus data,
		// or whether to log non-DogLog NT data to a log file
		DogLog.setOptions(new DogLogOptions().withNtPublish(false).withLogExtras(false).withCaptureNt(true));
		// doglog can optionally (by default) log using a separate thread to improve performance
		// however, this thread could take time (3-5s) to start up and will
		// incur the time start up penalty the first time DogLog.log() is called
		// doglog log thread takes some time to start up so warm it up
		DogLog.log("", "");

		// this is what starts the log file writer (I think doglog does this as
		// well, but I added doglog way after these and never removed it)
		DataLogManager.start();
		// this will log driverstation data (robot mode, disabled/enabled, etc)
		// to the log file
		DriverStation.startDataLog(DataLogManager.getLog());
		// this configures epilogue which is the main logging library used
		// by adding annotations (the @Logged) to classes, functions, fields, etc,
		// we can automatically log all the values in such classes to NT Tables
		// or whatever backend we specify
		// the main configuration option to use is the config.backend, which can
		// be used to log to NT (the default) or to the log file
		// you can also configure the period of logging to either slow down or 
		// quicken the logging
		Epilogue.configure(config -> {
			// config.minimumImportance = Importance.CRITICAL;
			config.minimumImportance = Importance.DEBUG;
			config.backend = new FileBackend(DataLogManager.getLog());
		});
		// this starts up epilogue
		Epilogue.bind(this);
		// DogLog.setEnabled(false);

		// this is the match data we sent to the drivers to always show
		// the reason these are separate calls is because logging to NT is very
		// expensive and we are staggering all the logging calls and separating
		// them into different periodic commands
		addPeriodic(() -> DogLog.forceNt.log("Match/Shift Time Left", m_robotContainer.shifttimeleft()), 0.5);
		addPeriodic(() -> DogLog.forceNt.log("Match/Hub Active", m_robotContainer.isHubActive()), 0.5);
		addPeriodic(() -> DogLog.forceNt.log("Match/Hub Next Active", m_robotContainer.shiftNext()), 0.5);
		addPeriodic(() -> DogLog.forceNt.log("Match/Is Transition", m_robotContainer.isTransitionPeriod()), 1);
		addPeriodic(() -> DogLog.forceNt.log("Match/Is Shift 1", m_robotContainer.isShift1()), 1);
		addPeriodic(() -> DogLog.forceNt.log("Match/Is Shift 2", m_robotContainer.isShift2()), 1);
		addPeriodic(() -> DogLog.forceNt.log("Match/Is Shift 3", m_robotContainer.isShift3()), 1);
		addPeriodic(() -> DogLog.forceNt.log("Match/Is Shift 4", m_robotContainer.isShift4()), 1);
		addPeriodic(() -> DogLog.forceNt.log("Match/Is Endgame", m_robotContainer.isEndGame()), 1);

		disabledTimer.restart();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if (DriverStation.isEnabled()) {
			disabledTimer.reset();
		}
		if (RobotController.getBatteryVoltage() < lowBatteryVoltage
				&& disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
			Leds.getInstance().lowbattery = true;

			if (RobotController.getBatteryVoltage() < criticallyLowBatteryVoltage)
				Leds.getInstance().criticallyLowbattery = true;
		}
	}

	@Override
	public void disabledInit() {
		NetworkTableInstance.getDefault().getTable(VisionConstants.Limelight4Name).getEntry("throttle_set")
				.setNumber(200);
		CommandScheduler.getInstance().schedule(
				Commands.runOnce(() -> {
					if (!DriverStation.isFMSAttached()) {
						FieldUtil.allianceWithActiveHubStart = null;
						FieldUtil.shift = 0;
					}
				}));
	}

	@Override
	public void disabledPeriodic() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
	}

	@Override
	public void disabledExit() {
		NetworkTableInstance.getDefault().getTable(VisionConstants.Limelight4Name).getEntry("throttle_set")
				.setNumber(0);
	}

	@Override
	public void autonomousInit() {
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
