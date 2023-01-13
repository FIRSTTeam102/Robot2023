package frc.robot.swerve;

import static frc.robot.Constants.loopPeriod_s;
import static frc.robot.Constants.SwerveConstants.*;

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
 * connected to the turn motor.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
	private FlywheelSim driveWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), driveGearRatio, 0.025);
	private FlywheelSim angleWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), angleGearRatio, 0.004);

	private double turnRelativePosition_rad = 0.0;
	private double turnAbsolutePosition_rad = Math.random() * 2.0 * Math.PI;
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;
	private boolean isDriveOpenLoop = true;
	private double driveSetpoint_mps = 0.0;
	private double angleSetpoint_deg = 0.0;

	private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(simDriveKs, simDriveKv, simDriveKa);
	private PIDController driveController = new PIDController(simDriveKp, simDriveKi, simDriveKd);
	private PIDController turnController = new PIDController(simAngleKp, simAngleKi, simAngleKd);

	/** Updates the set of inputs. */
	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// update the models
		driveWheelSim.update(loopPeriod_s);
		angleWheelSim.update(loopPeriod_s);

		// update the inputs that will be logged
		double angleDiffRad = angleWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s;
		turnRelativePosition_rad += angleDiffRad;
		turnAbsolutePosition_rad += angleDiffRad;
		while (turnAbsolutePosition_rad < 0)
			turnAbsolutePosition_rad += 2.0 * Math.PI;

		while (turnAbsolutePosition_rad > 2.0 * Math.PI)
			turnAbsolutePosition_rad -= 2.0 * Math.PI;

		inputs.drivePositionDeg = inputs.drivePositionDeg
			+ (driveWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s * (180.0 / Math.PI));

		inputs.driveDistanceMeters = inputs.driveDistanceMeters
			+ (driveWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s * wheelRadius_m);

		inputs.driveVelocityMetersPerSec = driveWheelSim.getAngularVelocityRadPerSec() * wheelRadius_m;

		inputs.driveAppliedPercentage = driveAppliedVolts / 12.0;
		inputs.driveCurrentAmps = new double[] {Math.abs(driveWheelSim.getCurrentDrawAmps())};
		// inputs.driveTempCelsius = new double[] {};

		inputs.angleAbsolutePositionDeg = turnAbsolutePosition_rad * (180.0 / Math.PI);
		inputs.anglePositionDeg = turnRelativePosition_rad * (180.0 / Math.PI);
		inputs.angleVelocityRevPerMin = angleWheelSim.getAngularVelocityRadPerSec() * (60.0 / (2.0 * Math.PI));

		inputs.angleAppliedPercentage = turnAppliedVolts / 12.0;
		inputs.angleCurrentAmps = new double[] {Math.abs(angleWheelSim.getCurrentDrawAmps())};
		// inputs.angleTempCelsius = new double[] {};

		// calculate and apply the "on-board" controllers for the turn and drive motors
		turnAppliedVolts = turnController.calculate(turnRelativePosition_rad, angleSetpoint_deg * (Math.PI / 180.0));
		turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
		angleWheelSim.setInputVoltage(turnAppliedVolts);

		if (!isDriveOpenLoop) {
			double velocityRadPerSec = driveSetpoint_mps / wheelRadius_m;
			driveAppliedVolts = feedForward.calculate(velocityRadPerSec)
				+ driveController.calculate(inputs.driveVelocityMetersPerSec, velocityRadPerSec);
			driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
			driveWheelSim.setInputVoltage(driveAppliedVolts);
		}

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(driveWheelSim.getCurrentDrawAmps(),
				angleWheelSim.getCurrentDrawAmps()));
	}

	/** Run the drive motor at the specified percentage of full power. */
	@Override
	public void setDriveMotorPercentage(double percentage) {
		isDriveOpenLoop = true;
		driveController.reset();
		driveAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
		driveWheelSim.setInputVoltage(driveAppliedVolts);
	}

	/** Run the drive motor at the specified velocity. */
	@Override
	public void setDriveVelocity(double velocity) {
		isDriveOpenLoop = false;
		driveSetpoint_mps = velocity;
	}

	/** Run the turn motor to the specified angle. */
	@Override
	public void setAnglePosition(double degrees) {
		angleSetpoint_deg = degrees;
	}
}
