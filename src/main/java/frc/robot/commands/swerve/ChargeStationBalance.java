package frc.robot.commands.swerve;

import frc.robot.constants.SwerveConstants;
import frc.robot.io.GyroIOInputsAutoLogged;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

// angles are between +/- pi

public class ChargeStationBalance extends CommandBase {
	private final double maxSpeed_mps = 0.1 * SwerveConstants.maxVelocity_mps;
	// private final double maxTurn = 0.1 * SwerveConstants.maxAngularVelocity_radps;

	// private final double maxAngularVelocity_radps = 0.04;

	// stops driving when within @fieldcal
	private final double maxAngle_rad = Units.degreesToRadians(4);

	private final PIDController driveController = new PIDController(0.3, 0, 0.01);

	private Swerve swerve;
	private GyroIOInputsAutoLogged gyro;

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		this.gyro = swerve.gyroInputs;
		addRequirements(swerve);

		// SmartDashboard.putData("balance drive pid", driveController);
		driveController.setTolerance(maxAngle_rad);
		driveController.setSetpoint(0);
		driveController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		// state = State.turn; // always reset state
		// swerve.stop();
	}

	// // stores the current state of the state machine
	// // private enum State {
	// // turn, balance
	// // }

	// // State state = State.turn;

	// private boolean angleZeroed(double angle_rad) {
	// return minAngle_rad < angle_rad && angle_rad < maxAngle_rad;
	// }

	// private double sign(double input) {
	// return input > 0.0 ? 1.0 : -1.0;
	// }

	@Override
	public void execute() {
		var outputSpeed = driveController.calculate(swerve.getTilt());

		if (outputSpeed > maxSpeed_mps) {
			outputSpeed = maxSpeed_mps;
		}

		swerve.drive(new Translation2d(outputSpeed, new Rotation2d(0)), 0, true);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return driveController.atSetpoint();
	}

	// // todo:, also don't pass borders as parameters
	// public boolean geofence(double blueTopY, double blueBottomY, double blueLeftX, double blueRightX,
	// double redTopY, double redBottomY, double redLeftX, double redRightX) {
	// return (swerve.translationY > blueTopY - SwerveConstants.robotfence
	// && swerve.translationY < blueBottomY - SwerveConstants.robotfence
	// && swerve.translationX > blueLeftX - SwerveConstants.robotfence
	// && swerve.translationX < blueRightX - SwerveConstants.robotfence)
	// || (swerve.translationY > redTopY - SwerveConstants.robotfence
	// && swerve.translationY < redBottomY - SwerveConstants.robotfence
	// && swerve.translationX > redLeftX - SwerveConstants.robotfence
	// && swerve.translationX < redRightX - SwerveConstants.robotfence);
	// }
}
