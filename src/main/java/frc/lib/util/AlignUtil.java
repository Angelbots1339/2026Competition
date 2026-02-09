package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlignConstants;
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

	public static final Transform2d towerAlignStart = new Transform2d(0.3, 0, Rotation2d.kZero);

	public static Pose2d getTargetTower(Swerve swerve) {
		Pose2d target = swerve.getPose().nearest(Arrays.asList(
				FieldUtil.getTowerCenter().transformBy(leftTowerOffset),
				FieldUtil.getTowerCenter().transformBy(rightTowerOffset)));
		return target;
	}

	public static Command ppPathFind(Pose2d target) {
		return AutoBuilder.pathfindToPose(target, AlignConstants.ppConstraints);
	}

	// this needs to be defered so we build the target at run time
	public static Command driveToClimbPosition(Swerve swerve) {
		Pose2d target = getTargetTower(swerve);
		Pose2d alignStart = getTargetTower(swerve).plus(towerAlignStart);
		return Commands.sequence(
				ppPathFind(alignStart),
				// PP doesn' tguarentee correct heading so run pidtopose in place to make sure
				// heading is correct
				swerve.pidtoPose(() -> alignStart),
				swerve.pidtoPose(() -> target));
	}
}