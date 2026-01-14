package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class Constants {
	public class DriverConstants {
		public static final int DriverPort = 0;

		public static final double joystickDeadband = 0.1;

		public static double joystickDeadband(double val, boolean curveInputs) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), curveInputs ? 2 : 1),
					joystickDeadband) * Math.signum(val);
		}
	}

	public class RobotConstants {
		public static final double maxSpeed = 1;
		public static final double maxRot = Math.PI * 2;
		public static final Distance length = Inches.of(32.25);
		public static final Distance width = Inches.of(32.25);

		public static final Translation2d climberOffset = new Translation2d(length.div(4), width.div(4).unaryMinus());

		public static final double angularDriveKP = 0.075;
		public static final double angularDriveKI = 0;
		public static final double angularDriveKD = 0.005;
		public static final double angularDriveKS = 0.4; // radians per sec
		public static final double angularDriveTolerance = 1.5; // Degrees

		public static final double pidToPoseKP = 2.5;
		public static final double pidToPoseKD = 0;
		public static final double pidToPoseKS = 0.15;
		public static final double pidToPoseTolerance = 0.03; // Meters
		public static final double pidToPoseMaxSpeed = 1; // Meters per second
	}

	public class VisionConstants {
		public static final String LimelightName = "limelight";
	}
}
