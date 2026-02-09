package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Swerve;

public class AlignUtil {
	/*
	 * right/left = prospective of driver
	 * transformation relative to robot and direction of pose = rotation of robot
	 * positive = left, forward of robot from front
	 * rotation = 0 = facing the pose directly with the front of the robot (the face
	 * of the pose will face the robot directly)
	 */
	public static final Transform2d leftTowerOffset = new Transform2d(Meters.zero(),
			FieldUtil.towerWidth.div(2).minus(RobotConstants.climberOffset.getMeasureX()),
			Rotation2d.kCCW_90deg);
	public static final Transform2d rightTowerOffset = new Transform2d(Meters.zero(),
			FieldUtil.towerWidth.div(2).unaryMinus().plus(RobotConstants.climberOffset.getMeasureX()),
			Rotation2d.kCW_90deg);

	public static Command driveToTowerSide(Swerve swerve) {
		return swerve.pidtoPose(
				() -> swerve.getPose().nearest(Arrays.asList(
						FieldUtil.getTowerCenter().transformBy(leftTowerOffset),
						FieldUtil.getTowerCenter().transformBy(rightTowerOffset))));
	}
}