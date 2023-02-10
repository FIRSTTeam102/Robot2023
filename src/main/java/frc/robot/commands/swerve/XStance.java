package frc.robot.commands.swerve;

import static frc.robot.constants.SwerveConstants.*;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * sets the swerve modules in the x-stance orientation.
 * the wheels are aligned to make an 'X',
 * which makes it more difficult for other robots to push us.
 */
public class XStance extends CommandBase {
	private Swerve swerve;
	private SwerveModuleState[] states;

	public XStance(Swerve swerve) {
		addRequirements(swerve);
		this.swerve = swerve;

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		states = swerve.kinematics.toSwerveModuleStates(chassisSpeeds, swerve.getCenterRotation());
		states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(trackWidth_m / wheelBase_m));
		states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[3].angle = new Rotation2d(Math.PI * 3.0 / 2.0 - Math.atan(trackWidth_m / wheelBase_m));
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		// todo: do we want closed loop?
		swerve.setModuleStates(states, true, true);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
