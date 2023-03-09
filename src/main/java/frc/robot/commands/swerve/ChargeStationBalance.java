package frc.robot.commands.swerve;

import frc.robot.constants.SwerveConstants;
import frc.robot.io.GyroIOInputsAutoLogged;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.littletonrobotics.junction.Logger;

// angles are between +/- pi

public class ChargeStationBalance extends CommandBase {
	private final double maxSpeed_mps = 0.1 * SwerveConstants.maxVelocity_mps;
	// private final double maxTurn = 0.1 * SwerveConstants.maxAngularVelocity_radps;

	private final double maxAngularVelocity_radps = 0.15;

	// stops driving when within
	private final double maxAngle_rad = Units.degreesToRadians(3);
	private final double minAngle_rad = -maxAngle_rad;

	// private final PIDController driveController = new PIDController(0.1, 0, 0.0005);

	private Swerve swerve;
	private GyroIOInputsAutoLogged gyro;

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		this.gyro = swerve.gyroInputs;
		addRequirements(swerve);

		// SmartDashboard.putData("balance drive pid", driveController);
		// driveController.setTolerance(0.05, 0.05);
		// driveController.setSetpoint(0);
		// driveController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		state = State.turn; // always reset state
		swerve.stop();
		angle_rad = Double.POSITIVE_INFINITY;
	}

	// stores the current state of the state machine
	private enum State {
		turn, balance
	}

	State state = State.turn;

	private boolean angleZeroed(double angle_rad) {
		return minAngle_rad < angle_rad && angle_rad < maxAngle_rad;
	}

	private double sign(double input) {
		return input > 0.0 ? 1.0 : -1.0;
	}

	double angle_rad;

	@Override
	/**
	 * @see https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/commands/AutoBalance.java
	 */
	public void execute() {
		angle_rad = swerve.getYaw().getSin() * gyro.pitch_rad
			+ swerve.getYaw().getCos() * gyro.roll_rad;
		Logger.getInstance().recordOutput("Balance/angle_rad", angle_rad);

		double angleVelocity_radps = swerve.getYaw().getSin() * gyro.pitch_radps
			+ swerve.getYaw().getCos() * gyro.roll_radps;

		boolean shouldStop = angle_rad < 0 && angleVelocity_radps > maxAngularVelocity_radps
			|| angle_rad > 0 && angleVelocity_radps < -maxAngularVelocity_radps;

		if (shouldStop)
			swerve.stop();
		else
			swerve.drive(new Translation2d(
				maxSpeed_mps * sign(angle_rad),
				0), 0, true);
	}

	// @formatter:off
	/*public void execute() {
		boolean pitchZeroed = angleZeroed(gyro.pitch_rad),
			rollZeroed = angleZeroed(gyro.roll_rad);

		// already balanced for now
		if (pitchZeroed && rollZeroed) {
			swerve.stop();
			return;
		}

		double shortestAngle_rad = (Math.abs(gyro.pitch_rad) < Math.abs(gyro.roll_rad)) ? gyro.pitch_rad : gyro.roll_rad;

		// todo: move side to side to take up left space
		// will these move in the right direction?
		// track which way we came up? or see which direction yaw is pointed in?

		switch (state) {
			// todo:
			case turn:
				if (pitchZeroed || rollZeroed)
					// move on
					state = State.balance;
				else {
					swerve.drive(new Translation2d(0, 0), maxTurn * sign(shortestAngle_rad), false);
					break;
				}
			case balance:
				if (!pitchZeroed && !rollZeroed)
					state = State.turn;

				// fixme: when gyro is actually mounted fix the axises
				double x = pitchZeroed ? 0 : driveController.calculate(gyro.pitch_rad),
				y = rollZeroed ? 0 : driveController.calculate(gyro.roll_rad);
				swerve.drive(new Translation2d(maxSpeed * x, maxSpeed * y), 0.0, false);
		}
		
		Logger.getInstance().recordOutput("Balance/state",state.toString());
	}*/
	// @formatter:on

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public boolean geofence(double blueTopY, double blueBottomY, double blueLeftX, double blueRightX,
		double redTopY, double redBottomY, double redLeftX, double redRightX) {
		return (swerve.translationY > blueTopY - SwerveConstants.robotfence
			&& swerve.translationY < blueBottomY - SwerveConstants.robotfence
			&& swerve.translationX > blueLeftX - SwerveConstants.robotfence
			&& swerve.translationX < blueRightX - SwerveConstants.robotfence)
			|| (swerve.translationY > redTopY - SwerveConstants.robotfence
				&& swerve.translationY < redBottomY - SwerveConstants.robotfence
				&& swerve.translationX > redLeftX - SwerveConstants.robotfence
				&& swerve.translationX < redRightX - SwerveConstants.robotfence);
	}
}
