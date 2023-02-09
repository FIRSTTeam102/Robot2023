package frc.robot.commands.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStationBalance extends CommandBase {
	private final double maxSpeed = 0.2 * SwerveConstants.maxVelocity_mps;
	private final double maxTurn = 0.2 * SwerveConstants.maxAngularVelocity_radps;

	Swerve swerve;

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	private double pitch_rad;
	private double roll_rad;

	@Override
	public void initialize() {
		state = State.turn; // always reset state
		swerve.stop();
	}

	// stores the current state of the state machine
	private enum State {
		turn, balance
	}

	State state = State.turn;

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
		// angles are between +/- pi
		pitch_rad = swerve.getPitch_rad();
		roll_rad = swerve.getRoll_rad();

		boolean pitchZeroed = withinDeadband(pitch_rad),
			rollZeroed = withinDeadband(roll_rad);

		// already balanced for now
		if (pitchZeroed && rollZeroed)
			return;

		double shortestDistance = (Math.abs(pitch_rad) < Math.abs(roll_rad)) ? pitch_rad : roll_rad;

		// todo: pid? to better aim instead of constant speed
		// todo: move side to side to take up left space
		// will these move in the right direction?
		// do we have to deal with field orientation?
		// track which way we came up? or see which direction yaw is pointed in?

		switch (state) {
			case turn:
				if (pitchZeroed || rollZeroed)
					// move on
					state = State.balance;
				else {
					swerve.drive(new Translation2d(0, 0), maxTurn * sign(shortestDistance));
					break;
				}
			case balance:
				if (!pitchZeroed && !rollZeroed)
					state = State.turn;

				// fixme: when gyro is actually mounted fix the axises
				double x = pitchZeroed ? 0.0 : -sign(pitch_rad),
					y = rollZeroed ? 0.0 : -sign(roll_rad);

				swerve.drive(new Translation2d(maxSpeed * x, maxSpeed * y), 0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		// these won't be seen in the ui but you can see them in the tree
		builder.addStringProperty("state", () -> state.toString(), null);
		builder.addDoubleProperty("pitch", () -> swerve.getPitch_rad(), null);
		builder.addDoubleProperty("roll", () -> swerve.getRoll_rad(), null);
	}
}
