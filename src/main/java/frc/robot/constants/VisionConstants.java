package frc.robot.constants;

public final class VisionConstants {
	/* AprilTagVision */
	public static final double maxZDistanceAprilTag_m = 1.8934684 /* grid to charge station */
		- 0.5334; /* limelight to front bumper */
	public static final double poseError_m = 1; // comparing visionPose to pose

	/* RetroreflectiveVision */
	public static final double rotateKp = 0;
	public static final double rotateKd = 0;
	public static final double crosshairTargetBoundRotateX_rad = 0.035;

	/* ObjectDetectionVision */
	public static final double crosshairObjectBoundTranslateZ_m = 0.559;
	public static final double crosshairObjectBoundRotateX_rad = 0.035;
};
