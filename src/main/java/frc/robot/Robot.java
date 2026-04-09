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
		m_robotContainer = new RobotContainer();
		// PathfindingCommand.warmupCommand().schedule();
		DogLog.setOptions(new DogLogOptions().withNtPublish(false).withLogExtras(false).withCaptureNt(true));
		// doglog log thread takes some time to start up so warm it up
		DogLog.log("", "");

		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		Epilogue.configure(config -> {
			// config.minimumImportance = Importance.CRITICAL;
			config.minimumImportance = Importance.DEBUG;
			config.backend = new FileBackend(DataLogManager.getLog());
		});
		Epilogue.bind(this);
		// DogLog.setEnabled(false);

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
