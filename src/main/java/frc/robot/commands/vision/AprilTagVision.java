package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.FieldVisionPipeline;
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
		// Sets pipeline to AprilTag
		vision.setFieldVisionPipeline(FieldVisionPipeline.AprilTag);

		// If see an AprilTag and are within the community, AprilTagVision will execute
		regeneratePaths = (vision.inputs.fieldVisionBotpose_FieldspaceTranslationX_m < VisionConstants.botpose_fieldBlueCommunityGeoFenceX_m
			|| vision.inputs.fieldVisionBotpose_FieldspaceTranslationX_m > VisionConstants.botpose_fieldRedCommunityGeoFenceX_m);
		cancelPPCommand();
	}

	@Override
	public void execute() {
		// If isPipelineReady true and regeneratePaths true, begin execute
		if (!vision.isPipelineReady() || !regeneratePaths) {
			return;
		}

		// When see a grid AprilTag or grid doublesubstation AprilTag, we will position and rotate to alliance color's side
		switch (routine) {
			case BlueRedGridDoublesubstationLeft -> {
				switch ((int) vision.inputs.fieldVisionTargetAprilTag) {
					case 5 -> {
						System.out.println("Swerve --> BlueDoublesubstation5Left");
						botpose_fieldGoToX_m = 0.76 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 7.5; // flipped
					}
					case 4 -> {
						System.out.println("Swerve --> RedDoublesubstation4Left");
						botpose_fieldGoToX_m = 15.77 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 6.02; // flipped
					}

					case 6 -> {
						System.out.println("Swerve --> BlueGrid6Left");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 3.86;
					}
					case 7 -> {
						System.out.println("Swerve --> BlueGrid7Left");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 2.16;
					}
					case 8 -> {
						System.out.println("Swerve --> BlueGrid8Left");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 0.50;
					}

					case 3 -> {
						System.out.println("Swerve --> RedGrid3Left");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 4.97;
					}
					case 2 -> {
						System.out.println("Swerve --> RedGrid2Left");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 3.29;
					}
					case 1 -> {
						System.out.println("Swerve --> RedGrid1Left");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 1.60;
					}
				}
			}

			case BlueRedGridMiddle -> {
				switch ((int) vision.inputs.fieldVisionTargetAprilTag) {
					case 6 -> {
						System.out.println("Swerve --> BlueGrid6Middle");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 4.42;
					}
					case 7 -> {
						System.out.println("Swerve --> BlueGrid7Middle");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 2.74;
					}
					case 8 -> {
						System.out.println("Swerve --> BlueGrid8Middle");
						botpose_fieldGoToX_m = 1.80 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 1.06;
					}

					case 3 -> {
						System.out.println("Swerve --> RedGrid3Middle");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 4.42;
					}
					case 2 -> {
						System.out.println("Swerve --> RedGrid2Middle");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 2.75;
					}
					case 1 -> {
						System.out.println("Swerve --> RedGrid1Middle");
						botpose_fieldGoToX_m = 14.73 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 1.05;
					}

					// double substation, do nothing for middle
					default -> {
						return;
					}
				}
			}

			case BlueRedGridDoublesubstationRight -> {
				switch ((int) vision.inputs.fieldVisionTargetAprilTag) {
					case 5 -> {
						System.out.println("Swerve --> BlueDoublesubstation5Right");
						botpose_fieldGoToX_m = 0.76 + VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 6.02; // flipped
					}
					case 4 -> {
						System.out.println("Swerve --> RedDoublesubstation4Right");
						botpose_fieldGoToX_m = 15.77 - VisionConstants.botpose_fieldOffsetX_m;
						botpose_fieldGoToY_m = 7.5; // flipped
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
			}

			default -> {
				return;
			}
		}

		// Generate a one-time path using from pose2d to AprilTag
		var rotation = Rotation2d.fromDegrees(switch ((int) vision.inputs.fieldVisionTargetAprilTag) {
			case 1, 2, 3, 4 -> 0; // red side (facing right)
			case 5, 6, 7, 8 -> 180; // blue side (facing left)
			default -> 0; // hopefully never
		});
		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
			new PathConstraints(3, 2),
			new PathPoint(swerve.getPose().getTranslation(), swerve.getPose().getRotation(), swerve.getPose().getRotation()),
			new PathPoint(new Translation2d(botpose_fieldGoToX_m, botpose_fieldGoToY_m), rotation, rotation));

		cancelPPCommand();
		ppCommand = new PathPlannerCommand(trajectory, swerve, false);
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
