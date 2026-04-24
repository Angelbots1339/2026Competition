package frc.robot.regression;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.FieldUtil;
import frc.robot.subsystems.Swerve;

public class ShooterRegression {
	public static final double minDistance = 2.0;
	// this contains all the data used to map distance to shooter/spinner rps
	public static final double[][] shotRPSData = {
			// distance (m), shooter rps, spinner rps
			// { 1.828, 41, 3 },
			// { 2.315, 43, 7 },
			// { 2.614, 43, 10 },
			// { 2.953, 45, 10 },
			// { 3.317, 45.5, 12 },
			// { 3.550, 46, 13 },
			// { 3.90, 47, 15 },

			// new shooter data
			// { 2.132, 38, 5.5 },
			// { 2.57, 38.5, 7.5 },
			// { 2.99, 40, 9.5 },
			// { 3.53, 42, 10 },
			// { 3.97, 43, 12 },
			// { 4.60, 44, 14 },
			// { 5.12, 46, 16 },

			// new new shooter data
			{ 2.19, 37, 4 },
			{ 2.52, 38, 5.5 },
			{ 2.76, 39, 7 },
			{ 3.05, 40, 8 },
			{ 3.31, 40, 9 },
			{ 3.64, 41, 10 },
			{ 4.23, 42, 12 },
			{ 5.38, 45, 15 },
	};

	public static final double[][] tofData = {
			// distance, tof from ball leaving the shooter to the hub

			// 19, 24, 22, 20, 22, 19
			{ 2.03, 21.0 / 30.0 },

			// 27, 23, 27, 28, 27, 28, 26, 26, 28
			{ 2.52, 26.667 / 30.0 },

			// todo: count actually
			// 29, 26, 31, 29, 28
			{ 3.01, 28.6 / 30.0 },

			{ 3.52, 32 / 28.99 },

			{ 4.04, 35 / 30.0 },

	};

	// this is an interpolatingtree map which basically acts as a map/set
	// in which if a value other than the valuse in the table are called, it interpolates
	// from the two nearest data points to return a value
	public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

	// this is the same thing as the timeofFlightMap but using a custom return value
	// of a double[]
	// we basically define a custom interpolation function used by the 
	// interpolatingTreeMap class
	public static final InterpolatingTreeMap<Double, double[]> shotRPSMap = new InterpolatingTreeMap<Double, double[]>(
			InverseInterpolator.forDouble(), new Interpolator<double[]>() {
				public double[] interpolate(double[] startValue, double[] endValue, double t) {
					double[] new_vals = { MathUtil.interpolate(startValue[0], endValue[0], t),
							MathUtil.interpolate(startValue[1], endValue[1], t), };
					return new_vals;
				};
			});

	// static {} blocks allow us to write code that runs in the class itself
	// the reason we are separating the actual data and the interpolation maps
	// is because it is easier to add data to the double[]
	static {
		for (double[] data : shotRPSData) {
			shotRPSMap.put(data[0], new double[] { data[1], data[2] });
		}
		for (double[] data : tofData) {
			timeOfFlightMap.put(data[0], data[1]);
		}
	}

	public record ShooterParams(Rotation2d angle, double shooterRPS, double spinnerRPS, Angle maxAngleError,
			boolean isValid) {
	};

	// this is the function that takes the swerves current position and velocity
	// and returns the rps and angle of the robot to accurately shoot into the hub
	public static ShooterParams getShotParams(Swerve swerve) {
		Translation2d target = FieldUtil.getHubCenter().getTranslation();
		Translation2d pose = swerve.getPose().getTranslation();
		double distance = target.getDistance(pose);

		ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
				swerve.getYaw());
		double vx = fieldSpeeds.vxMetersPerSecond;
		double vy = fieldSpeeds.vyMetersPerSecond;
		// when shooting while the robot is moving, the robot's velocity is imparted
		// on the ball, either slowing or quickining its velocity.
		// as such, we are able target an offset position such that after shooting,
		// the robot's imparted velocity accurately moves the ball to hit the target
		//
		// we do this by offsetting the target by the robot's current velocity vector
		// scaled up by how long it takes for the ball to enter the hub after leaving
		// the robot (the TOF)

		// When shooting, we are assuming the TOF of the ball being the same as when
		// leaving the current pose.
		// however, as we are shooting farther/closer depending on our
		// velocity/lookahead pose, the actual TOF is shorter/longer
		// iterate and use the TOF from the lookaheadpose to calculate a better
		// lookahead pose
		Translation2d lookaheadPose = pose;
		double lookaheadDistance = distance;
		double tof = 0;
		for (int i = 0; i < 3; i++) {
			tof = timeOfFlightMap.get(lookaheadDistance);
			lookaheadPose = pose.plus(new Translation2d(vx * tof, vy * tof));
			lookaheadDistance = target.getDistance(lookaheadPose);
		}

		double[] rps = shotRPSMap.get(lookaheadDistance);

		Rotation2d angle = Rotation2d.fromRadians(
				target.minus(lookaheadPose).getAngle().plus(Rotation2d.k180deg).getRadians());

		// this angle is the maximum angle we can be off of before we have the center of the
		// robot face away from the hub funnel
		Angle maxAngleError = Radians
				.of(Math.abs(Math.atan2(FieldUtil.hubRadius.in(Meters), distance)));

		DogLog.log("Regression/Lookahead Pose", new Pose2d(lookaheadPose, angle));
		DogLog.log("Regression/Max Angle Error", maxAngleError);
		DogLog.log("Regression/Angle", angle);
		DogLog.log("Regression/Shooter RPS", rps[0]);
		DogLog.log("Regression/Spinner RPS", rps[1]);
		// basically don't allow the driver to shoot balls if we are too close to the robot
		// looking back at this, this should probably be just distance, since the 
		// blocker from shooting closer was the angle of our shot being too shallow
		// and hitting the hub.  shooting from farther away would increase the angle
		// of our shot and possibly allow for a theoritally sotm shot to hit inside
		boolean isValid = lookaheadDistance >= minDistance;

		return new ShooterParams(angle, rps[0], rps[1], maxAngleError, isValid);
	}

	public static double[] getRegressionRPS(double meters) {
		return shotRPSMap.get(meters);
	}

}
