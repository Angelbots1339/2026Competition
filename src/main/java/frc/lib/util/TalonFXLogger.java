package frc.lib.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
	public TalonFXLogger() {
		super(TalonFX.class);
	}

	@Override
	public void update(EpilogueBackend backend, TalonFX motor) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			backend.log("CanID", motor.getDeviceID());
			backend.log("Applied Voltage (V)", motor.getMotorVoltage().getValueAsDouble());
			backend.log("Current (A)", motor.getStatorCurrent().getValueAsDouble());
			backend.log("Current Acceleration (rps/s)", motor.getAcceleration().getValueAsDouble());
			backend.log("Current Velocity (rps)", motor.getVelocity().getValueAsDouble());
			backend.log("Encoder Position (Rotations)", motor.getPosition().getValueAsDouble());
		}
	}
}
