package frc.lib.choreo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity BumpPassVel = Units.MetersPerSecond.of(2);
    public static final Distance FieldHeight = Units.Meters.of(8.069);

    public static final class Poses {
        public static final Pose2d DepotIntake = new Pose2d(0.75, 5.002, Rotation2d.fromRadians(2.234));
        public static final Pose2d DepotIntakeEnd = new Pose2d(0.75, 6.71, Rotation2d.fromRadians(2.234));
        public static final Pose2d DepotShoot = new Pose2d(2.322, 5.397, Rotation2d.fromRadians(-0.588));
        public static final Pose2d FirstBumpStart = new Pose2d(3.74, 5.822, Rotation2d.fromRadians(-1.571));
        public static final Pose2d HubLeftStart = new Pose2d(3.565, 5.04, Rotation2d.fromRadians(1.571));
        public static final Pose2d LeftNeutralFarmStop = new Pose2d(7.764, 5.01, Rotation2d.fromRadians(-1.571));
        public static final Pose2d LeftNeutralStart = new Pose2d(7.764, 6.841, Rotation2d.fromRadians(-1.571));
        public static final Pose2d LeftNeutralStop = new Pose2d(7.764, 5.531, Rotation2d.fromRadians(-1.571));
        public static final Pose2d LeftBumpStart = new Pose2d(3.643, 6.062, Rotation2d.fromRadians(-3.142));
        public static final Pose2d SecondBumpStart = new Pose2d(5.603, 5.562, Rotation2d.fromRadians(-1.571));
        public static final Pose2d SecondBumpStop = new Pose2d(3.651, 5.562, Rotation2d.fromRadians(-1.571));
        public static final Pose2d NeutralShoot = new Pose2d(3.485, 5.458, Rotation2d.fromRadians(2.237));

        private Poses() {}
    }

    private ChoreoVars() {}
}