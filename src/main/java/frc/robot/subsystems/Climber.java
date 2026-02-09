package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.tuning.ClimberTuning;
import frc.robot.Constants.ClimberConstants;

@Logged
public class Climber extends SubsystemBase {
	private TalonFX climberMotor = new TalonFX(ClimberConstants.ClimberMotorPort);

	private Distance targetPosition = Meters.zero();

	public Climber() {
		climberMotor.getConfigurator().apply(ClimberConstants.ClimberMotorConfig);
		// make sure the climber is fully extended to the second sharpie mark
		climberMotor.setPosition(0);
	}

	public void climb() {
		setClimberPosition(ClimberConstants.ClimbPosition);
	}

	public void home() {
		setClimberPosition(ClimberConstants.HomePosition);
	}

	public void disable() {
		climberMotor.setControl(new NeutralOut());
	}

	public void setVoltage(double volts) {
		climberMotor.setControl(new VoltageOut(volts));
	}

	public Distance getClimberPosition() {
		return Meters.of(climberMotor.getPosition().getValueAsDouble());
	}

	public void resetClimberPosition() {
		climberMotor.setPosition(0);
	}

	public void setClimberPosition(Distance position) {
		targetPosition = position;
		climberMotor.setControl(new PositionTorqueCurrentFOC(position.in(Meters)));
	}

	public void logPID() {
		ClimberTuning.createPID("Climber/Climber PID", climberMotor, ClimberConstants.ClimberMotorConfig);
	}

	@Override
	public void periodic() {
	}
}
