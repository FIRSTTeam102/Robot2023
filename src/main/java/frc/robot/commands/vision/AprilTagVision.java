package frc.robot.commands.vision;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.elevator.SetElevatorPosition;
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
	private Elevator elevator;
	private Swerve swerve;
	private PathPlannerCommand ppCommand;
	private boolean regeneratePaths = true;
	private double xDistanceGoTo_m;
	private double yDistanceGoTo_m;

	public enum Routine {
		BlueRedGridDoublesubstationLeft, BlueRedGridMiddle, BlueRedGridDoublesubstationRight
	}

	public AprilTagVision(Routine routine, Vision vision, Elevator elevator, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.elevator = elevator;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		// Sets pipeline to Apriltag
		vision.setPipeline(Pipeline.AprilTag);

		// If we are too far away maxZDistanceAprilTag_m from the Apriltag, AprilTagVision will not execute
		regeneratePaths = vision.inputs.botpose_targetspaceTranslationZ_m < VisionConstants.maxZDistanceAprilTag_m;
		cancelPPCommand();
	}

	@Override
	public void execute() {
		// If isPipelineReady true and regeneratePaths true, begin execute
		if (!vision.isPipelineReady() || !regeneratePaths) {
			return;
		}

		// When we see a grid apriltag, we will position and rotate to alliance color's side
		switch (routine) {
			case BlueRedGridDoublesubstationLeft:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Swerve --> BlueDoublesubstation5Left");
					xDistanceGoTo_m = 0.70;
					yDistanceGoTo_m = 6.13;
					new SetElevatorPosition(elevator, ElevatorConstants.doubleSubstationHeight_m);
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Swerve --> RedDoublesubstation4Left");
					xDistanceGoTo_m = 15.84;
					yDistanceGoTo_m = 7.33;
					new SetElevatorPosition(elevator, ElevatorConstants.doubleSubstationHeight_m);
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Left");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 3.87;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Left");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 2.19;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Left");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 0.51;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Left");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 4.97;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Left");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 3.29;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Left");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 1.61;
				}
				break;

			case BlueRedGridMiddle:
				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Middle");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 4.42;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Middle");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 2.75;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Middle");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 1.06;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Middle");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 4.40;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Middle");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 2.74;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Middle");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 1.05;
				}
				break;

			case BlueRedGridDoublesubstationRight:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Swerve --> BlueDoublesubstation5Right");
					xDistanceGoTo_m = 0.70;
					yDistanceGoTo_m = 7.47;
					new SetElevatorPosition(elevator, ElevatorConstants.doubleSubstationHeight_m);
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Swerve --> RedDoublesubstation4Right");
					xDistanceGoTo_m = 15.84;
					yDistanceGoTo_m = 6.00;
					new SetElevatorPosition(elevator, ElevatorConstants.doubleSubstationHeight_m);
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Swerve --> BlueGrid6Right");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 4.98;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Swerve --> BlueGrid7Right");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 3.86;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Swerve --> BlueGrid8Right");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 1.62;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Swerve --> RedGrid3Right");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 3.85;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Swerve --> RedGrid2Right");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 2.17;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Swerve --> RedGrid1Right");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 0.49;
				}
				break;

			default:
				return;
		}

		// Generate a path using from pose2d to apriltag
		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
			new PathConstraints(AutoConstants.maxVelocity_mps, AutoConstants.maxAcceleration_mps2),
			new PathPoint(swerve.getPose().getTranslation(), swerve.getPose().getRotation()), // start at current pos
			new PathPoint(new Translation2d(xDistanceGoTo_m, yDistanceGoTo_m),
				Rotation2d.fromRadians(0)));

		cancelPPCommand();
		ppCommand = new PathPlannerCommand(trajectory, swerve, false);
		ppCommand.schedule();
	}

	// Stop swerve
	@Override
	public void end(boolean interrupted) {
		cancelPPCommand();
	}

	// AprilTagVision.java does not end automatically if PathPlannerCommand.java is running
	@Override
	public boolean isFinished() {
		if (ppCommand != null && !ppCommand.isFinished())
			return false;

		return false;
	}

	private void cancelPPCommand() {
		if (ppCommand != null)
			ppCommand.cancel();
	}
}
