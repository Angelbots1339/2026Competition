package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Meters;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Climber;

public class ClimberTuning {
	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Shooter);

	private static Trigger runClimber = baseTrigger.and(() -> tester.getYButton());

	private static Distance targetPosition = Meters.zero();

	public static void init(Climber climber) {
		DogLog.tunable("Climber/target", 0.0, target -> targetPosition = Meters.of(target));
		runClimber.whileTrue(
				Commands.run(() -> climber.setClimberPosition(targetPosition), climber)
						.handleInterrupt(() -> climber.disable()));
	}
}
