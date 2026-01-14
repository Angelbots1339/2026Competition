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

	public static Pose2d BlueHubCenter = new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);
	public static Pose2d RedHubCenter = new Pose2d(Meter.convertFrom(651.22 - 182.11, Inches),
			Meter.convertFrom(158.84, Inches), Rotation2d.k180deg);

	public static Pose2d BlueTowerCenter = new Pose2d(Meter.convertFrom(156.06 - 114.26, Inches),
			Meter.convertFrom(25.62 + 132.70 - 11.46, Inches), Rotation2d.kZero);
	public static Pose2d RedTowerCenter = new Pose2d(Meter.convertFrom(651.22 - 156.06 + 114.26, Inches),
			Meter.convertFrom(87.46 + 82.32, Inches), Rotation2d.k180deg);

	public static Distance towerWidth = Inches.of(47.00);

	public static enum HubShiftTime {
		TRANSITION_SHIFT_START(140),
		TRANSITION_SHIFT_END(130),
		SHIFT_1_START(130),
		SHIFT_1_END(105),
		SHIFT_2_START(105),
		SHIFT_2_END(80),
		SHIFT_3_START(80),
		SHIFT_3_END(55),
		SHIFT_4_START(55),
		SHIFT_4_END(30),
		ENDGAME_START(30),
		;

		private double time;

		private HubShiftTime(double time) {
			this.time = time;
		}

	}

	public static Alliance allianceWithActiveHubStart = null;

	public static Alliance getAlliance() {
		Optional<Alliance> alliance = DriverStation.getAlliance();

		if (alliance.isEmpty())
			return null;

		return alliance.get();
	}

	public static boolean isRedAlliance() {
		Alliance alliance = getAlliance();

		if (alliance == Alliance.Red) {
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

	public static void getShiftOrder() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.length() <= 0)
			return;

		char state = gameData.charAt(0);

		/*
		 * FMS sends the alliance whose hub is inactivated first
		 * https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html#
		 * data-format
		 */
		switch (state) {
			case 'R':
				allianceWithActiveHubStart = Alliance.Blue;
				break;
			case 'B':
				allianceWithActiveHubStart = Alliance.Red;
				break;
			default:
				break;
		}
	}

	/*
	 * 20sec: Auto: both enabled
	 * 10sec: Transition: both enabled
	 * 25sec: Shift 1: Auto win disabled
	 * 25sec: Shift 2: Auto win enabled
	 * 25sec: Shift 3: Auto win disabled
	 * 25sec: Shift 4: Auto win enabled
	 * 30sec: EndGame: both enabled
	 */
	public static boolean isHubActive() {
		double matchTime = (int) DriverStation.getMatchTime();

		if (DriverStation.isAutonomous())
			return true;

		if (!DriverStation.isTeleop())
			return false;

		if (matchTime >= HubShiftTime.TRANSITION_SHIFT_END.time)
			return true;

		if (matchTime >= HubShiftTime.SHIFT_1_END.time)
			return getAlliance() == allianceWithActiveHubStart;

		if (matchTime >= HubShiftTime.SHIFT_2_END.time)
			return getAlliance() != allianceWithActiveHubStart;

		if (matchTime >= HubShiftTime.SHIFT_3_END.time)
			return getAlliance() == allianceWithActiveHubStart;

		if (matchTime >= HubShiftTime.SHIFT_4_END.time)
			return getAlliance() != allianceWithActiveHubStart;

		if (matchTime <= HubShiftTime.ENDGAME_START.time)
			return true;

		return false;
	}

	public static int getShiftTimeLeft() {
		int matchTime = (int) DriverStation.getMatchTime();
		// excludes auto as well
		if (matchTime <= HubShiftTime.ENDGAME_START.time) {
			return -1;
		}

		if (matchTime >= HubShiftTime.TRANSITION_SHIFT_END.time) {
			return (int) (matchTime - HubShiftTime.TRANSITION_SHIFT_END.time);
		}

		matchTime -= HubShiftTime.ENDGAME_START.time;

		return matchTime % 25;
	}
}
