package frc.lib.util;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Swerve;

public class AlignUtil {
	/*
	 * right/left = prospective of driver
	 * transformation relative to robot and direction of pose = rotation of robot
	 * positive = left, forward of robot from front
	 * rotation = 0 = facing the pose directly with the front of the robot
	 */
	public static final Transform2d rightTowerOffset = new Transform2d(
			RobotConstants.climberOffset.getMeasureX().unaryMinus(),
			FieldUtil.towerWidth.div(2).unaryMinus().plus(RobotConstants.climberOffset.getMeasureY()),
			Rotation2d.kZero);
	public static final Transform2d leftTowerOffset = new Transform2d(
			RobotConstants.climberOffset.getMeasureX().unaryMinus(),
			FieldUtil.towerWidth.div(2).unaryMinus().plus(RobotConstants.climberOffset.getMeasureY()).unaryMinus(),
			Rotation2d.k180deg);

	/* offsets a pose target with a robot relative offset */
	public static Pose2d offsetPose(Pose2d target, Transform2d offset) {
		// convert robot relative to blue origin coordinates
		offset = new Transform2d(-offset.getX(), -offset.getY(), offset.getRotation().plus(Rotation2d.k180deg));
		Translation2d tmp = offset.getTranslation();
		tmp.rotateBy(target.getRotation());

		target = target.plus(
				new Transform2d(tmp.getX(), tmp.getY(), offset.getRotation()));
		return target;
	}

	public static Command driveToTowerSide(Swerve swerve) {
		return swerve.pidtoPose(
				() -> swerve.getPose().nearest(Arrays.asList(offsetPose(FieldUtil.getTowerCenter(),
						leftTowerOffset),
						offsetPose(FieldUtil.getTowerCenter(), rightTowerOffset))));
	}
}