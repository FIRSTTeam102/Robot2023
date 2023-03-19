package frc.robot.commands.swerve;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.littletonrobotics.junction.Logger;

// angles are between +/- pi

public class ChargeStationBalance extends CommandBase {
	private final double maxSpeed_mps = 0.5;

	/** stops driving when within @fieldcal */
	private final double finshedAngle_rad = Units.degreesToRadians(4); // 3

	private final PIDController driveController = new PIDController(1.5, 0, 0.05);

	private Swerve swerve;

	private Timer finishedTimer = new Timer();

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);

		SmartDashboard.putData("balance drive pid", driveController);
		driveController.setTolerance(finshedAngle_rad);
		driveController.setSetpoint(0);
		driveController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		finishedTimer.reset();
	}

	@Override
	public void execute() {
		var tilt_rad = swerve.getTilt_rad();
		Logger.getInstance().recordOutput("Balance/tilt_rad", tilt_rad);

		var outputVelocity_mps = MathUtil.clamp(driveController.calculate(tilt_rad), -maxSpeed_mps, maxSpeed_mps);
		Logger.getInstance().recordOutput("Balance/outputVelocity_mps", outputVelocity_mps);

		if (driveController.atSetpoint()) {
			finishedTimer.start();

			swerve.stop();
		} else {
			finishedTimer.stop();
			finishedTimer.reset();

			swerve.drive(new Translation2d(outputVelocity_mps, 0), 0, true);
		}

		Logger.getInstance().recordOutput("Balance/finishedTimer_s", finishedTimer.get());
		Logger.getInstance().recordOutput("Balance/atSetpoint", driveController.atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		finishedTimer.stop();
	}

	@Override
	public boolean isFinished() {
		return finishedTimer.hasElapsed(0.6);
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
