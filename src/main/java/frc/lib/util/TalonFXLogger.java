package frc.lib.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

// this is an example of a custom logger for Epilogue, logging the TalonFX class
// which doesn't have a default logger.  anytime a TalonFX is asked to be logged
// either through an explicit @Logged annotation or through its class its a field
// of, this logger will be used.
//
// note, only the first defined logger will be used.  for example, if @Logged
// is already used on a loggable class (ex: the swerve class which has loggable functions)
// defining a custom logger class will not override it
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
			backend.log("Stator Current (A)", motor.getStatorCurrent().getValueAsDouble());
			backend.log("Supply Current (A)", motor.getSupplyCurrent().getValueAsDouble());
			backend.log("Current Acceleration (rpss)", motor.getAcceleration().getValueAsDouble());
			backend.log("Current Velocity (rps)", motor.getVelocity().getValueAsDouble());
			backend.log("Encoder Position (Rotations)", motor.getPosition().getValueAsDouble());
		}
	}
}
