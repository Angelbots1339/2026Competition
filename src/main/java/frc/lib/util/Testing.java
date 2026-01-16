package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.Swerve;

public class Testing {

	private XboxController tester = new XboxController(DriverConstants.TesterPort);

	private Trigger baseTrigger = new Trigger(() -> DriverStation.isTestEnabled());
	private Trigger characterizeSwerveRadius = baseTrigger.and(() -> tester.getAButton());

	private Swerve swerve;

	public Testing(Swerve swerve) {
		this.swerve = swerve;

		bindTriggers();
	}

	public void bindTriggers() {
		characterizeSwerveRadius.whileTrue(swerve.characterizeWheelRadius());
	}
}
