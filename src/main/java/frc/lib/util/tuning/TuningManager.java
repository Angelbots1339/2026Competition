package frc.lib.util.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TuningManager {
	public static TuningMode tuningMode = TuningMode.Shooter;

	public static SendableChooser<TuningMode> tuningModeChooser = new SendableChooser<TuningMode>();

	static {
		tuningModeChooser.setDefaultOption("Shooter", TuningMode.Shooter);
		tuningModeChooser.addOption("Swerve", TuningMode.Swerve);
		tuningModeChooser.addOption("Intake", TuningMode.Intake);
		tuningModeChooser.addOption("Climber", TuningMode.Climber);
		SmartDashboard.putData(tuningModeChooser);
	}

	public static void init(Swerve swerve, Shooter shooter, Intake intake, Climber climber) {
		if (swerve != null)
			SwerveTuning.init(swerve);
		if (shooter != null)
			ShooterTuning.init(shooter);
		if (intake != null)
			IntakeTuning.init(intake);
		if (climber != null)
			ClimberTuning.init(climber);
	}

	public static void changeMode() {
		tuningMode = tuningModeChooser.getSelected();
	}
}
