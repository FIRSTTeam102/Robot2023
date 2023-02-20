package frc.robot.commands.swerve;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStationBalance extends CommandBase {
	private final double maxSpeed = 0.005 * SwerveConstants.maxVelocity_mps;
	private final double maxTurn = 0.005 * SwerveConstants.maxAngularVelocity_radps;

	private final PIDController driveController = new PIDController(0.1, 0, 0.0005);

	Swerve swerve;

	public void geofence(double bluetopy, double bluebottomy, double blueleftx, double bluerightx, double redtopy,
		double redbottomy, double redleftx, double redrightx) {
		if (swerve.translationY > bluetopy - SwerveConstants.robotfence
			&& swerve.translationY < bluebottomy - SwerveConstants.robotfence
			&& swerve.translationX > blueleftx - SwerveConstants.robotfence
			&& swerve.translationX < bluerightx - SwerveConstants.robotfence
			&& swerve.translationY > redtopy - SwerveConstants.robotfence
			&& swerve.translationY < redbottomy - SwerveConstants.robotfence
			&& swerve.translationX > redleftx - SwerveConstants.robotfence
			&& swerve.translationX < redrightx - SwerveConstants.robotfence) {

		} else {

		}
	}

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);

		SmartDashboard.putData("balance drive pid", driveController);

		driveController.setTolerance(0.05, 0.05);
		driveController.setSetpoint(0);
		driveController.enableContinuousInput(-Math.PI, Math.PI);
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
		return withinDeadband(input, 0.03);
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
		if (pitchZeroed && rollZeroed) {
			swerve.stop();
			return;
		}

		double shortestAngle_rad = (Math.abs(pitch_rad) < Math.abs(roll_rad)) ? pitch_rad : roll_rad;

		// todo: pid? to better aim instead of constant speed
		// todo: move side to side to take up left space
		// will these move in the right direction?
		// track which way we came up? or see which direction yaw is pointed in?

		switch (state) {
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
				double x = pitchZeroed ? 0 : driveController.calculate(pitch_rad),
					y = rollZeroed ? 0 : driveController.calculate(roll_rad);

				swerve.drive(new Translation2d(maxSpeed * x, maxSpeed * y), 0.0, false);
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
