package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStationBalance extends CommandBase {
	Swerve swerve;

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	private double pitch_rad;
	private double roll_rad;

	@Override
	public void initialize() {
		state = State.turn;
		swerve.stop();
	}

	// stores the current state of the state machine
	private enum State {
		turn, balance
	}

	State state;

	/**
	 * pitch = rotating around side-to-side axis
	 * yaw = rotating around vertical axis
	 * roll = rotating around front-to-back axis
	 */
	private enum Dimension {
		pitch, yaw, roll
	}

	private boolean withinDeadband(double input, double deadband) {
		return Math.abs(input) < deadband;
	}

	private boolean withinDeadband(double input) {
		return withinDeadband(input, 0.05);
	}

	private double sign(double input) {
		return input > 0.0 ? 1.0 : -1.0;
	}

	@Override
	public void execute() {
		pitch_rad = swerve.getPitch_rad();
		roll_rad = swerve.getRoll_rad();

		boolean pitchZeroed = withinDeadband(pitch_rad),
			rollZeroed = withinDeadband(roll_rad);

		// already balanced for now
		if (pitchZeroed && rollZeroed)
			return;

		double shortestDistance = (Math.abs(pitch_rad) < Math.abs(roll_rad)) ? pitch_rad : roll_rad;

		// todo: pid? to better aim isntead of constant speed
		// todo: move side to side to take up left space

		switch (state) {
			case turn:
				if (pitchZeroed || rollZeroed)
					// move on
					state = State.balance;
				else {
					swerve.drive(new Translation2d(0, 0),
						SwerveConstants.maxAngularVelocity_radps * 0.2 * sign(shortestDistance));
					break;
				}
			case balance:
				if (!pitchZeroed && !rollZeroed)
					state = State.turn;

				// fixme: when gyro is actually mounted fix the axises
				double x = pitchZeroed ? 0.0 : -sign(pitch_rad),
					y = rollZeroed ? 0.0 : -sign(roll_rad);

				swerve.drive(new Translation2d(
					SwerveConstants.maxVelocity_mps * 0.2 * x,
					SwerveConstants.maxVelocity_mps * 0.2 * y), 0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
