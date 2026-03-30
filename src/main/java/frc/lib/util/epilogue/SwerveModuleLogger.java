package frc.lib.util.epilogue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.lib.util.TalonFXLogger;

@CustomLoggerFor(SwerveModule.class)
public class SwerveModuleLogger extends ClassSpecificLogger<SwerveModule> {
	TalonFXLogger driveMotorLogger = new TalonFXLogger();
	TalonFXLogger steerMotorLogger = new TalonFXLogger();

	public SwerveModuleLogger() {
		super(SwerveModule.class);
	}

	@Override
	public void update(EpilogueBackend backend, SwerveModule module) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			driveMotorLogger.update(backend.getNested("Drive Motor"), (TalonFX) module.getDriveMotor());
			steerMotorLogger.update(backend.getNested("Steer Motor"), (TalonFX) module.getSteerMotor());
		}
	}
}
