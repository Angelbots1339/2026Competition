package frc.lib.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.Shooter;

public class ShooterTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(() -> DriverStation.isTestEnabled());
	private static Trigger runVoltage = baseTrigger.and(() -> tester.getYButton());
	private static Trigger pidtune = baseTrigger.and(() -> tester.getAButton());
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());
	private static Trigger characterizeVelocity = baseTrigger.and(() -> tester.getBButton());
	private static SysIdRoutine velocitySysIDRoutine = null;

	private static Shooter shooter;

	private static AngularVelocity targetAngle = RotationsPerSecond.zero();

	public static void init(Shooter shooter) {
		ShooterTuning.shooter = shooter;
		shooter.logTuning();
		setupSysId();

		runVoltage.whileTrue(Commands.run(() -> {
			shooter.setVoltage();
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));
		pidtune.onTrue(Commands.runOnce(() -> {
			shooter.leader.getConfigurator().apply(shooter.getPID());
			targetAngle = RotationsPerSecond.of(SmartDashboard.getNumber(TuningConstants.Shooter.velocityNTName, 0));
		}));
		pidtune.whileTrue(Commands.run(() -> {
			shooter.setVelocity(targetAngle);
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));
		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setVelocityFOC(targetAngle);
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));

		characterizeVelocity.whileTrue(Commands.sequence(
				Commands.runOnce(SignalLogger::start),
				sysIdQuasistatic(velocitySysIDRoutine, Direction.kForward),
				Commands.waitSeconds(2),
				sysIdQuasistatic(velocitySysIDRoutine, Direction.kReverse),
				Commands.waitSeconds(2),
				sysIdDynamic(velocitySysIDRoutine, Direction.kForward),
				Commands.waitSeconds(2),
				sysIdDynamic(velocitySysIDRoutine, Direction.kReverse),
				Commands.runOnce(SignalLogger::stop)).handleInterrupt(SignalLogger::stop));
	}

	public static void setupSysId() {
		velocitySysIDRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
						null, null,
						Seconds.of(7), // Use default timeout (10 s)
						// Log state with SignalLogger class
						state -> SignalLogger.writeString("SysIdVelocity_State", state.toString())),
				new SysIdRoutine.Mechanism(
						output -> {
							SignalLogger.writeDouble("Voltage", output.in(Volts));
						},
						null,
						shooter));
	}

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public static Command sysIdQuasistatic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public static Command sysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}
}
