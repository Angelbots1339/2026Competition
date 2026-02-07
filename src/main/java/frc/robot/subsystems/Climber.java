package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
	private TalonFX climberMotor = new TalonFX(ClimberConstants.ClimberMotorPort);

	public Climber() {
		climberMotor.getConfigurator().apply(ClimberConstants.ClimberMotorConfig);
		climberMotor.setPosition(0);
	}

	public Distance getClimberPosition() {
		return Meters.of(climberMotor.getPosition().getValueAsDouble());
	}

	public void setClimberPosition(Distance position) {
		climberMotor.setControl(new PositionTorqueCurrentFOC(position.in(Meters)));
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

	@Override
	public void periodic() {
	}
}
