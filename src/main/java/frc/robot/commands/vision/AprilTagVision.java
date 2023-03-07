package frc.robot.commands.vision;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.swerve.PathPlannerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AprilTagVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;
	private PathPlannerCommand ppCommand;
	private boolean regeneratePaths = true;
	private double botpose_fieldGoToX_m;
	private double botpose_fieldGoToY_m;

	public enum Routine {
		BlueRedGridDoublesubstationLeft, BlueRedGridMiddle, BlueRedGridDoublesubstationRight
	}

	public AprilTagVision(Routine routine, Vision vision, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		// Sets pipeline to Apriltag
		vision.setPipeline(Pipeline.AprilTag);

		// If see an Apriltag and are within maxZDistanceAprilTag_m from the Apriltag, AprilTagVision will execute
		regeneratePaths = vision.inputs.target == true
			&& vision.inputs.botpose_fieldTranslationZ_m < VisionConstants.maxZDistanceAprilTag_m;
		cancelPPCommand();
	}

	@Override
	public void execute() {
		// If isPipelineReady true and regeneratePaths true, begin execute
		if (!vision.isPipelineReady() || !regeneratePaths) {
			return;
		}

		// When we see a grid apriltag or grid doublesubstation, we will position and rotate to alliance color's side
		switch (routine) {
			case BlueRedGridDoublesubstationLeft:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Swerve --> BlueDoublesubstation5Left");
					botpose_fieldGoToX_m = 0.70;
					botpose_fieldGoToY_m = 6.13;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Swerve --> RedDoublesubstation4Left");
					botpose_fieldGoToX_m = 15.84;
					botpose_fieldGoToY_m = 7.33;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Left");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 3.87;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Left");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 2.19;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Left");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 0.51;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Left");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 4.97;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Left");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 3.29;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Left");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 1.61;
				}
				break;

			case BlueRedGridMiddle:
				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Middle");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 4.42;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Middle");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 2.75;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Middle");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 1.06;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Middle");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 4.40;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Middle");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 2.74;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Middle");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 1.05;
				}
				break;

			case BlueRedGridDoublesubstationRight:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Swerve --> BlueDoublesubstation5Right");
					botpose_fieldGoToX_m = 0.70;
					botpose_fieldGoToY_m = 7.47;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Swerve --> RedDoublesubstation4Right");
					botpose_fieldGoToX_m = 15.84;
					botpose_fieldGoToY_m = 6.00;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Right");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 4.98;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Right");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 3.86;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Right");
					botpose_fieldGoToX_m = 1.80;
					botpose_fieldGoToY_m = 1.62;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Right");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 3.85;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Right");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 2.17;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Right");
					botpose_fieldGoToX_m = 14.73;
					botpose_fieldGoToY_m = 0.49;
				}
				break;

			default:
				return;
		}

		// Generate a path using from pose2d to apriltag
		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
			new PathConstraints(AutoConstants.maxVelocity_mps, AutoConstants.maxAcceleration_mps2),
			new PathPoint(swerve.getPose().getTranslation(), swerve.getPose().getRotation(), swerve.getPose().getRotation()),
			new PathPoint(new Translation2d(botpose_fieldGoToX_m, botpose_fieldGoToY_m),
				Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)));

		cancelPPCommand();
		ppCommand = new PathPlannerCommand(trajectory, swerve, true);
		ppCommand.schedule();

		regeneratePaths = false;
	}

	// Stop swerve
	@Override
	public void end(boolean interrupted) {
		cancelPPCommand();
	}

	// AprilTagVision.java does not end automatically if PathPlannerCommand.java is running
	@Override
	public boolean isFinished() {
		try {
			if (ppCommand != null && !ppCommand.isFinished())
				return false;
		} catch (NullPointerException e) {
			// The command's initialize is not called before the first time that our isFinished is called
			return false;
		}

		return false;
	}

	private void cancelPPCommand() {
		if (ppCommand != null)
			ppCommand.cancel();
	}
}
