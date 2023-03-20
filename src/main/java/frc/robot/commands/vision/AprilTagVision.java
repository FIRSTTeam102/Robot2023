package frc.robot.commands.vision;

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
	private boolean inAuto;
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
		this.inAuto = false;
	}

	public AprilTagVision(Routine routine, Vision vision, Swerve swerve, boolean inAuto) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
		this.inAuto = inAuto;
	}

	@Override
	public void initialize() {
		// Sets pipeline to Apriltag
		vision.setPipeline(Pipeline.AprilTag);

		// If see an Apriltag and are within the community, AprilTagVision will execute
		regeneratePaths = vision.inputs.target == true
			&& (vision.inputs.botpose_fieldTranslationX_m < VisionConstants.botpose_fieldBlueCommunityGeoFenceX_m
				|| vision.inputs.botpose_fieldTranslationX_m > VisionConstants.botpose_fieldRedCommunityGeoFenceX_m);
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
					botpose_fieldGoToX_m = 0.76 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 6.21;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Swerve --> RedDoublesubstation4Left");
					botpose_fieldGoToX_m = 15.77 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 7.33;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Left");
					botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 3.86;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Left");
					botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 2.16;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Left");
					botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 0.50;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Left");
					botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 4.97;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Left");
					botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 3.29;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Left");
					botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 1.60;
				}
				break;

			case BlueRedGridMiddle:
				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Middle");
					botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 4.42;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Middle");
					botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 2.74;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Middle");
					botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 1.06;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Middle");
					botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 4.42;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Middle");
					botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 2.75;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Middle");
					botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
					botpose_fieldGoToY_m = 1.05;
				}
				break;

			case BlueRedGridDoublesubstationRight:
				switch ((int) vision.inputs.targetAprilTag) {
					case 5 -> {
						System.out.println("Swerve --> BlueDoublesubstation5Right");
						botpose_fieldGoToX_m = 0.76 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 7.33;
					}
					case 4 -> {
						System.out.println("Swerve --> RedDoublesubstation4Right");
						botpose_fieldGoToX_m = 15.77 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 6.21;
					}

					case 6 -> {
						System.out.println("Swerve --> BlueGrid6Right");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 4.98;
					}
					case 7 -> {
						System.out.println("Swerve --> BlueGrid7Right");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 3.30;
					}
					case 8 -> {
						System.out.println("Swerve --> BlueGrid8Right");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 1.61;
					}

					case 3 -> {
						System.out.println("Swerve --> RedGrid3Right");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 3.86;
					}
					case 2 -> {
						System.out.println("Swerve --> RedGrid2Right");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 2.16;
					}
					case 1 -> {
						System.out.println("Swerve --> RedGrid1Right");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 0.49;
					}
				}
				break;

			default:
				return;
		}

		// Generate a one-time path using from pose2d to apriltag
		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
			new PathConstraints(2.0, 0.5),
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
		swerve.stop();
	}

	// AprilTagVision.java does not end automatically if PathPlannerCommand.java is running
	@Override
	public boolean isFinished() {
		try {
			if (inAuto && ppCommand != null && ppCommand.isFinished())
				return true;

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
