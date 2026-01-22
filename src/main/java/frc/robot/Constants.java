package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
	public class DriverConstants {
		public static final int DriverPort = 0;
		public static final int TesterPort = 2;

		public static final double joystickDeadband = 0.1;

		public static double joystickDeadband(double val, boolean curveInputs) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), curveInputs ? 2 : 1),
					joystickDeadband) * Math.signum(val);
		}
	}

	public class RobotConstants {
		public static final LinearVelocity maxSpeed = MetersPerSecond.of(1);
		public static final AngularVelocity maxRot = RadiansPerSecond.of(Math.PI * 2);
		public static final Distance length = Inches.of(32.25);
		public static final Distance width = Inches.of(32.25);

		public static final Translation2d climberOffset = new Translation2d(length.div(4), Meters.zero());

		// has least amount of error, overshooting seems to be result of wheel slip on
		// concrete
		public static final double angularDriveKP = 6.78;
		public static final double angularDriveKI = 0;
		public static final double angularDriveKD = 0.1;
		public static final double angularDriveKS = 0;
		public static final double angularDriveKV = 0;
		public static final SimpleMotorFeedforward angularDriveFeedforward = new SimpleMotorFeedforward(angularDriveKS,
				angularDriveKV);
		public static final TrapezoidProfile.Constraints angularDriveConstraints = new TrapezoidProfile.Constraints(
				10,
				20);
		public static final Angle angularDriveTolerance = Degrees.of(0.25); // Degrees

		public static final double pidToPoseKP = 2.5;
		public static final double pidToPoseKD = 0;
		public static final double pidToPoseKS = 0.15;
		public static final Distance pidToPoseTolerance = Meters.of(0.03); // Meters
		public static final LinearVelocity pidToPoseMaxSpeed = MetersPerSecond.of(1); // Meters per second
	}

	public class VisionConstants {
		public static final String LimelightName = "limelight";

		public static double calcStdDev(double metersFromTarget) {
			return 0.08 * Math.pow(metersFromTarget, 2);
		}
	}

	public class TuningConstants {
		public static final String tuningNTPrefix = "Tuning/";

		public class Swerve {
			public static final String angularPIDNTName = tuningNTPrefix + "angular PID";
		}

		public class Shooter {
			public static final String voltageNTName = "voltage";
		}
	}

	public class ShooterConstants {
		public static final int LeaderPort = 1;
		public static final int FollowerPort = 0;
		public static final int Follower2Port = 2;
		public static TalonFXConfiguration config = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
				.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.5));
	}
}
