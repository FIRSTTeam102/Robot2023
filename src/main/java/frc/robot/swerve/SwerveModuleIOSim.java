package frc.robot.swerve;

import static frc.robot.constants.Constants.loopPeriod_s;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.Conversions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * A simulated version of the SwerveModuleIO interface.
 *
 * The swerve module is simulated as a flywheel connected to the drive motor and another flywheel
 * connected to the angle motor.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
	private FlywheelSim driveWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), driveGearRatio, 0.025);
	private FlywheelSim angleWheelSim = new FlywheelSim(DCMotor.getNEO(1), angleGearRatio, 0.004);

	private double angleAbsolutePosition_rad = 0.0; // Math.random() * Conversions.twoPi;
	private double driveAppliedVolts = 0.0;
	private double angleAppliedVolts = 0.0;
	private boolean isDriveOpenLoop = true;
	private double driveSetpoint_mps = 0.0;

	private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(simDriveKs, simDriveKv);
	private PIDController driveController = new PIDController(simDriveKp, simDriveKi, simDriveKd);

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// update the models
		driveWheelSim.update(loopPeriod_s);
		angleWheelSim.update(loopPeriod_s);

		// update the inputs that will be logged
		double angleDiffRad = angleWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s;
		inputs.anglePosition_rad += angleDiffRad;
		// using member so we start at a random location
		angleAbsolutePosition_rad += angleDiffRad;
		angleAbsolutePosition_rad = Conversions.angleModulus2pi(angleAbsolutePosition_rad + angleDiffRad);
		inputs.angleAbsolutePosition_rad = angleAbsolutePosition_rad;

		inputs.drivePosition_rad = inputs.drivePosition_rad
			+ driveWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s;
		inputs.driveDistance_m = inputs.driveDistance_m
			+ (driveWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s * wheelRadius_m);
		inputs.driveVelocity_mps = driveWheelSim.getAngularVelocityRadPerSec() * wheelRadius_m;
		inputs.driveAppliedPercentage = driveAppliedVolts / 12.0;
		inputs.driveCurrent_A = Math.abs(driveWheelSim.getCurrentDrawAmps());

		inputs.angleVelocity_radps = angleWheelSim.getAngularVelocityRadPerSec();
		inputs.angleAppliedPercentage = angleAppliedVolts / 12.0;
		inputs.angleCurrent_A = Math.abs(angleWheelSim.getCurrentDrawAmps());

		if (!isDriveOpenLoop) {
			double velocityRadPerSec = driveSetpoint_mps / wheelRadius_m;
			driveAppliedVolts = feedForward.calculate(velocityRadPerSec)
				+ driveController.calculate(inputs.driveVelocity_mps, velocityRadPerSec);
			driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
			driveWheelSim.setInputVoltage(driveAppliedVolts);
		}

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(driveWheelSim.getCurrentDrawAmps(),
				angleWheelSim.getCurrentDrawAmps()));
	}

	@Override
	public void setDriveMotorPercentage(double percentage) {
		isDriveOpenLoop = true;
		driveController.reset();
		driveAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
		driveWheelSim.setInputVoltage(driveAppliedVolts);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		isDriveOpenLoop = false;
		driveSetpoint_mps = velocity;
	}

	@Override
	public void setAngleVoltage(double voltage) {
		angleAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
		angleWheelSim.setInputVoltage(angleAppliedVolts);
	}

	@Override
	public double getCharacterizationVelocity_radps() {
		return driveWheelSim.getAngularVelocityRadPerSec();
	}

	@Override
	public void close() throws Exception {
		driveController.close();
	}
}
