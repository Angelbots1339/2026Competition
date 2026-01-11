package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
	public static Distance FieldWidth = Inches.of(651.22);
	public static Distance FieldHeight = Inches.of(317.69);

	public static Pose2d BlueHubCenter = new Pose2d(Meter.convertFrom(182.11, Inches),
			Meter.convertFrom(158.84, Inches), Rotation2d.kZero);
	public static Pose2d RedHubCenter = new Pose2d(Meter.convertFrom(651.22 - 182.11, Inches),
			Meter.convertFrom(158.84, Inches), Rotation2d.k180deg);

	public static Pose2d BlueTowerCenter = new Pose2d(Meter.convertFrom(156.06 - 114.26, Inches),
			Meter.convertFrom(25.62 + 132.70 - 11.46, Inches), Rotation2d.kZero);
	public static Pose2d RedTowerCenter = new Pose2d(Meter.convertFrom(651.22 - 156.06 + 114.26, Inches),
			Meter.convertFrom(87.46 + 82.32, Inches), Rotation2d.k180deg);

	public static boolean isRedAlliance() {
		Optional<Alliance> alliance = DriverStation.getAlliance();

		if (alliance.isEmpty())
			return false;

		if (alliance.get() == Alliance.Red) {
			return true;
		}

		return false;
	}

	public static Pose2d getHubCenter() {
		if (isRedAlliance()) {
			return RedHubCenter;
		}

		return BlueHubCenter;
	}

	public static Pose2d getTowerCenter() {
		if (isRedAlliance()) {
			return RedTowerCenter;
		}

		return BlueTowerCenter;
	}
}
