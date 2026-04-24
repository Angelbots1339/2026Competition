package frc.lib.util.tuning;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// this class manages the switching between tuning modes through a smartdashboard
// sendable chooser, enabling and disabling controller bindings for different
// subsystems / tuning modes
public class TuningManager {
	public static TuningMode tuningMode = TuningMode.Shooter;

	public static SendableChooser<TuningMode> tuningModeChooser = new SendableChooser<TuningMode>();

	public static void init(Swerve swerve, Shooter shooter, Intake intake, Indexer indexer) {
		tuningModeChooser.setDefaultOption("Shooter", TuningMode.Shooter);
		tuningModeChooser.addOption("Swerve", TuningMode.Swerve);
		tuningModeChooser.addOption("Regression", TuningMode.Regression);
		tuningModeChooser.addOption("Intake", TuningMode.Intake);
		tuningModeChooser.addOption("Indexer", TuningMode.Indexer);
		SmartDashboard.putData("Tuning Mode", tuningModeChooser);

		SwerveTuning.init(swerve);
		ShooterTuning.init(shooter, indexer);
		RegressionTuning.init(swerve, shooter, indexer, intake);
		IntakeTuning.init(intake);
		IndexerTuning.init(indexer);
	}

	public static void changeMode() {
		if (frc.robot.Constants.useTesting) {
			tuningMode = tuningModeChooser.getSelected();
		}
	}

	// this is a function used to create tunable values for the PID of motors
	// that way, we can easily tweak pid valuse without redeploying code or using
	// phoenix tuner
	public static void createPID(String key, TalonFX motor, TalonFXConfiguration config) {
		// DogLog tunables are values sent to NT which are listened to for changes
		// and perform a callback with the value of changes
		// in this case, everytime a value changes, we change the pid valuse
		// with the value returned from NT
		DogLog.tunable(key + "/kP", config.Slot0.kP,
				newP -> motor.getConfigurator().apply(config.Slot0.withKP(newP)));
		DogLog.tunable(key + "/kI", config.Slot0.kI,
				newI -> motor.getConfigurator().apply(config.Slot0.withKI(newI)));
		DogLog.tunable(key + "/kD", config.Slot0.kD,
				newD -> motor.getConfigurator().apply(config.Slot0.withKD(newD)));
		DogLog.tunable(key + "/kS", config.Slot0.kS,
				newS -> motor.getConfigurator().apply(config.Slot0.withKS(newS)));
		DogLog.tunable(key + "/kV", config.Slot0.kV,
				newV -> motor.getConfigurator().apply(config.Slot0.withKV(newV)));
		DogLog.tunable(key + "/kG", config.Slot0.kG,
				newG -> motor.getConfigurator().apply(config.Slot0.withKG(newG)));
	}
}
