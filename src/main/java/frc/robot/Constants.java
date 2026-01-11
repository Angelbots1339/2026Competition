package frc.robot;

import edu.wpi.first.math.MathUtil;

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
		public static final double maxSpeed = 5;
		public static final double maxRot = Math.PI * 2;
	}
}
