package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Autos {
	private AutoFactory factory;
	private Swerve swerve;

	public Autos(Swerve swerve) {
		this.swerve = swerve;
		factory = new AutoFactory(
				swerve::getPose, // A function that returns the current field-relative Pose2d of the robot
				swerve::resetPose, // A function that receives a field-relative Pose2d to reset the robot's
									// odometry to.
				swerve.followChoreoPath(), // A function that receives the current SampleType and controls the
											// robot. More info below
				true, // If this is true, when on the red alliance, the path will be mirrored to the
						// opposite side, while keeping the same coordinate system origin.
				swerve); // The drive Subsystem to require for AutoTrajectory Commands.

		/* TODO: replace with actual commands */
		Command leftIntakeOpen = Commands.print("intake open").andThen(Commands.waitSeconds(1));
		Command leftIntakeClose = Commands.print("intake close").andThen(Commands.waitSeconds(1));

		Command passStart = Commands.print("pass start");
		Command passEnd = Commands.print("pass end");

		factory.bind("LeftIntakeStart", leftIntakeOpen);
		factory.bind("LeftIntakeEnd", leftIntakeClose);
		factory.bind("PassStart", passStart);
		factory.bind("PassEnd", passEnd);
	}

	public Command bumpTest() {
		final var routine = factory.newRoutine("bump test");
		final var bumptest = routine.trajectory("BumpTest");
		routine.active().onTrue(
				Commands.sequence(
						bumptest.resetOdometry(),
						bumptest.cmd()));
		// swerve.pointDrive(() -> 0.0, () -> 0.0, () -> FieldUtil.getHubCenter(), () ->
		// true)));

		return routine.cmd();
	}

	public Command bumpTestStraight() {
		final var routine = factory.newRoutine("bump test straight");
		final var bumptest = routine.trajectory("BumpTestStraight");
		routine.active().onTrue(
				Commands.sequence(
						bumptest.resetOdometry(),
						bumptest.cmd()));
		// swerve.pointDrive(() -> 0.0, () -> 0.0, () -> FieldUtil.getHubCenter(), () ->
		// true)));

		return routine.cmd();
	}

	public Command leftPassAuto() {
		final var routine = factory.newRoutine("Hub Depot Outpost Tower");
		final var leftPass = routine.trajectory("LeftPass");

		routine.active().onTrue(
				Commands.sequence(
						leftPass.resetOdometry(),
						leftPass.cmd()));

		return routine.cmd();
	}

	public Command hubDepotTowerAuto() {
		final var routine = factory.newRoutine("Hub Depot Outpost Tower");
		final var hubToDepotShoot = routine.trajectory("HubtoDepotShoot");
		final var depotShoottoTower = routine.trajectory("DepotShoottoTower");

		routine.active().onTrue(
				Commands.sequence(
						hubToDepotShoot.resetOdometry(),
						hubToDepotShoot.cmd(),
						depotShoottoTower.cmd()));

		return routine.cmd();
	}

	public Command hubDepotOutpostTowerAuto() {
		final var routine = factory.newRoutine("Hub Depot Outpost Tower");
		final var hubToDepotShoot = routine.trajectory("HubtoDepotShoot");
		final var depotShoottoOutpost = routine.trajectory("DepotShoottoOutpost");
		final var outpostToShoot = routine.trajectory("OutposttoShoot");
		final var outpostShoottoTower = routine.trajectory("OutpostShoottoTower");

		routine.active().onTrue(
				Commands.sequence(
						hubToDepotShoot.resetOdometry(),
						hubToDepotShoot.cmd(),
						depotShoottoOutpost.cmd(),
						outpostToShoot.cmd(),
						outpostShoottoTower.cmd()));

		return routine.cmd();
	}
}
