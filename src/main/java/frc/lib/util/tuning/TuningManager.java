package frc.lib.util.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TuningManager {
	public static TuningMode tuningMode = TuningMode.Shooter;

	public static SendableChooser<TuningMode> tuningModeChooser = new SendableChooser<TuningMode>();

	static {
		tuningModeChooser.setDefaultOption("Shooter", TuningMode.Shooter);
		tuningModeChooser.addOption("Swerve", TuningMode.Swerve);
		SmartDashboard.putData(tuningModeChooser);
	}

	public static void init(Swerve swerve, Shooter shooter) {
		if (swerve != null)
			SwerveTuning.init(swerve);
		if (shooter != null)
			ShooterTuning.init(shooter);
	}

	public static void changeMode() {
		tuningMode = tuningModeChooser.getSelected();
	}
}
