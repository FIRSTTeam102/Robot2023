package frc.robot;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import frc.robot.commands.Autos;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.grabber.GrabGrabber;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.grabber.StopGrabber;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.swerve.AngleOffsetCalibration;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.GamePieceVision;
import frc.robot.commands.vision.RetroreflectiveVision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.Getter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private static RobotContainer instance = null;

	public static RobotContainer getInstance() {
		if (instance == null)
			instance = new RobotContainer();
		return instance;
	}

	public final CommandXboxController driverController = new CommandXboxController(
		OperatorConstants.driverControllerPort);
	public final CommandGenericHID operatorConsole = new CommandGenericHID(
		OperatorConstants.operatorConsolePort);
	public final CommandJoystick operatorJoystick = new CommandJoystick(
		OperatorConstants.operatorJoystickPort);

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	public final Vision vision = new Vision();
	public final Swerve swerve = new Swerve(gyro, vision);
	public final Arm arm = new Arm();
	public final Elevator elevator = new Elevator();
	public final Grabber grabber = new Grabber();

	// these are suppliers b/c pathplanner gets the alliance in the constructor, but it's unknown until init
	private LoggedDashboardChooser<Supplier<Command>> autoChooser = new LoggedDashboardChooser<>("auto routine");
	// private GenericEntry autoDelay;

	@Getter
	private boolean lastDemoMode = true;
	public GenericEntry demoModeDash;
	private double demoSpeed = 0.5;
	private GenericEntry demoSpeedDash;
	@Getter
	private boolean lastDemoScoring = true;
	public GenericEntry demoScoringDash;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		new AngleOffsetCalibration(swerve);

		// setup autos
		// FW = field wall, LZ = loading zone barrier
		autoChooser.addOption("nothing", () -> new InstantCommand());
		autoChooser.addDefaultOption("cube and nothing", () -> Autos.initAndScore(this, ScoringPosition.HighCube));

		autoChooser.addOption("lz cube", () -> Autos.lzCube(this));
		autoChooser.addOption("lz cube 2", () -> Autos.lzCube2(this));
		autoChooser.addOption("lz cube aim cone", () -> Autos.lzCubeAimCone(this));
		autoChooser.addOption("lz cube pickup cone", () -> Autos.lzCubePickupCone(this));
		// autoChooser.addOption("lz 2cube balance", () -> Autos.lz2CubeBalance(this));

		autoChooser.addOption("fw cube", () -> Autos.fwCube(this));
		// autoChooser.addOption("fw cube balance", () -> Autos.fwCubeBalance(this));
		autoChooser.addOption("fw cube 2", () -> Autos.fwCube2(this));
		autoChooser.addOption("fw cube pickup cone", () -> Autos.fwCubePickupCone(this));
		// autoChooser.addOption("fw 2cube balance", () -> Autos.fw2CubeBalance(this));

		autoChooser.addOption("coop cube balance", () -> Autos.coopCubeBalance(this));
		autoChooser.addOption("coop cube mobility balance", () -> Autos.coopCubeMobilityBalance(this));

		// for testing
		autoChooser.addOption("[testing] drive characterization",
			() -> new FeedForwardCharacterization(swerve, true, swerve::runCharacterization,
				swerve::getCharacterizationVelocity));

		var driveTab = Shuffleboard.getTab(ShuffleboardConstants.driveTab);
		driveTab.add("auto routine", autoChooser.getSendableChooser())
			.withSize(4, 1).withPosition(0, 5);
		driveTab.add("alerts", Alert.getAlertsSendable())
			.withSize(5, 4).withPosition(4, 5);
		// autoDelay = driveTab.add("auto delay", 0.0)
		// .withSize(2, 1).withPosition(0, 7)
		// .getEntry();
		configureCameras();

		demoModeDash = driveTab.add("demo mode", lastDemoMode)
			.withWidget(BuiltInWidgets.kToggleSwitch)
			.withPosition(9, 5)
			.withSize(2, 1)
			.getEntry();
		demoScoringDash = driveTab.addPersistent("demo scoring", lastDemoScoring)
			.withWidget(BuiltInWidgets.kToggleSwitch)
			.withPosition(9, 6)
			.withSize(2, 1)
			.getEntry();
		demoSpeedDash = driveTab.addPersistent("swerve speed", demoSpeed)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0.0, "max", 1.0, "block increment", 0.1))
			.withPosition(11, 5)
			.withSize(4, 2)
			.getEntry();

		// clearButtons();
		// if (demoModeDash.getBoolean(lastDemoMode))
		// configureDemoButtons();
		// else
		// configureBindings();

		arm.setMaxSpeed(0.4);
		elevator.setMaxSpeed(0.25);

		elevator.setDefaultCommand(
			new SequentialCommandGroup(new SetScoringPosition(elevator, arm, ScoringPosition.AllIn, 0.4, 0.2),
				Commands.waitSeconds(1.5),
				new SetScoringPosition(elevator, arm, ScoringPosition.HolidayFun, AutoConstants.elevatorTolerance_m,
					AutoConstants.armTolerance_m),
				Commands.waitSeconds(3)));
	}

	public void updateDemoMode() {
		if (lastDemoMode != demoModeDash.getBoolean(lastDemoMode)) {
			lastDemoMode = demoModeDash.getBoolean(lastDemoMode);
			clearButtons();
			if (lastDemoMode)
				configureDemoButtons();
			else
				configureBindings();
		}
		if (lastDemoScoring != demoScoringDash.getBoolean(lastDemoScoring)) {
			lastDemoScoring = demoScoringDash.getBoolean(lastDemoScoring);
			if (lastDemoScoring) {
				// enable scoring
				elevator.pidController.setOutputRange(ElevatorConstants.minOutput, ElevatorConstants.maxOutput);
				arm.pidController.setOutputRange(ArmConstants.minOutput, ArmConstants.maxOutput);
			} else {
				// disable scoring, output range only affects pid stuff
				elevator.pidController.setOutputRange(0, 0);
				elevator.killMotor();
				arm.pidController.setOutputRange(0, 0);
				arm.killMotor();
				grabber.stop();
			}
		}
		if (lastDemoMode && demoSpeedDash.getDouble(demoSpeed) != demoSpeed) {
			demoSpeed = demoSpeedDash.getDouble(demoSpeed);
			if (demoTeleopSwerve != null)
				demoTeleopSwerve.newRateLimiter(Math.sqrt(demoSpeed)); // the less speedy, the less limitey
		}
	}

	public void clearButtons() {
		CommandScheduler.getInstance().cancelAll();
		CommandScheduler.getInstance().getActiveButtonLoop().clear();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController Xbox} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	public void configureBindings() {
		/*
		 * driver
		 */
		var teleopSwerve = new TeleopSwerve(
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX(),
			() -> -driverController.getRightX(),
			driverController.getHID()::getLeftBumper, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // preceise mode
			swerve, arm, elevator);
		swerve.setDefaultCommand(teleopSwerve);

		driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
			.whileTrue(teleopSwerve.holdToggleFieldRelative());
		driverController.rightBumper()
			.whileTrue(teleopSwerve.holdRotateAroundPiece());

		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());
		driverController.x().whileTrue(new XStance(swerve));

		var zeroYaw = teleopSwerve.new ZeroYaw();
		driverController.y().onTrue(zeroYaw);
		new Trigger(RobotController::getUserButton).onTrue(zeroYaw);

		driverController.start().whileTrue(new ChargeStationBalance(swerve)); // right menu

		driverController.pov(90) // left grid or left double substation
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationLeft, vision, swerve));
		driverController.pov(0) // middle grid
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridMiddle, vision, swerve));
		driverController.pov(270) // right grid or right double substation
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationRight, vision, swerve));

		driverController.pov(180)
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridLeftRight, vision, swerve));

		/*
		 * operator console
		 * G1 G2 G3 B4
		 * G5 Y6 Y7 B8
		 * R9 Y10 Y11 B12
		 * R13 R14 R15 B16
		 */

		// swerve to grid or double substation. swerve/arm/elevator game piece (green)
		operatorConsole.button(3) // left grid or left double substation
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationLeft, vision, swerve));
		operatorConsole.button(2) // middle grid
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridMiddle, vision, swerve));
		operatorConsole.button(1) // right grid or right double substation
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationRight, vision, swerve));

		// swerve or/and arm/elevator to grid or double substation (blue + orange)
		operatorConsole.button(4) // double substation
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.DoubleSubstation));
		operatorConsole.button(7) // high cone
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.HighCone))
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridLeftRight, vision, swerve));
		operatorConsole.button(8) // high cube
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.HighCube));
		operatorConsole.button(11) // mid cone
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.MidCone))
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridLeftRight, vision, swerve));
		operatorConsole.button(12) // mid cube
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.MidCube));
		operatorConsole.button(15) // all in
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.AllIn));
		operatorConsole.button(16) // ground
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.Ground));
		operatorConsole.button(10)
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.SingleSubstationCube));

		operatorConsole.button(5) // aim at game piece
			.whileTrue(new GamePieceVision(GamePieceVision.Routine.GamePieceGround, vision, swerve));
		operatorConsole.button(6) // autograb
			.whileTrue(Autos.intakeGroundUntimed(swerve, elevator, arm, grabber, vision));

		operatorConsole.button(14)
			.whileTrue(Commands.run(elevator::killMotor, elevator));
		// .whileTrue(arm.tempDisableLimits);

		// human player request
		operatorConsole.button(9).onTrue(new Lights.RequestGamePiece(Lights.Status.Cone));
		operatorConsole.button(13).onTrue(new Lights.RequestGamePiece(Lights.Status.Cube));

		/*
		 * operator flight stick
		 */
		// arm.setDefaultCommand(new ManualArmControl(arm, () -> -operatorJoystick.getX()));
		// elevator.setDefaultCommand(new ManualElevatorControl(elevator, () -> operatorJoystick.getY()));
		// todo: what happens when both pressed?
		operatorJoystick.trigger()
			.whileTrue(new GrabGrabber(grabber, GrabberConstants.cubeGrabSpeed));
		operatorJoystick.button(5)
			.whileTrue(new GrabGrabber(grabber, GrabberConstants.coneGrabSpeed));
		operatorJoystick.button(2)
			.whileTrue(new ReleaseGrabber(grabber));
		operatorJoystick.button(3)
			.onTrue(new MoveElevatorBy(elevator, ElevatorConstants.coneMoveDownHeight_m)
				.andThen(new ReleaseGrabber(grabber)));
		operatorJoystick.button(4)
			.onTrue(new StopGrabber(grabber));
		operatorJoystick.button(6)
			.whileTrue(Commands.startEnd(() -> grabber.move(-GrabberConstants.cubeShootSpeed), grabber::stop, grabber));
		// .onTrue(new GrabGrabberUntilGrabbed(grabber));
	}

	/** clamps axis to max demo speed */
	private double demoAxis(DoubleSupplier axis) {
		if (killed)
			return 0;
		// todo: clamp or multiply?
		return MathUtil.clamp(-axis.getAsDouble(), -demoSpeed, demoSpeed);
	}

	private double negSquare(double value) {
		return Math.copySign(Math.pow(value, 2), value);
	}

	private TeleopSwerve demoTeleopSwerve;

	public void configureDemoButtons() {
		/*
		 * driver
		 */
		demoTeleopSwerve = new TeleopSwerve(
			() -> demoAxis(driverController::getLeftY),
			() -> demoAxis(driverController::getLeftX),
			() -> demoAxis(() -> negSquare(driverController.getRightX())),
			() -> false, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // preceise mode
			swerve, arm, elevator);
		// demoTeleopSwerve.fieldRelative = false; // default to robot oriented
		demoTeleopSwerve.fieldRelative = true;
		swerve.setDefaultCommand(demoTeleopSwerve);

		// driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
		// .whileTrue(demoTeleopSwerve.holdToggleFieldRelative());
		// driverController.rightBumper()
		// .whileTrue(demoTeleopSwerve.holdRotateAroundPiece());

		driverController.a().onTrue(demoTeleopSwerve.toggleFieldRelative());
		driverController.x().whileTrue(new XStance(swerve));

		driverController.y().onTrue(demoTeleopSwerve.new ZeroYaw());

		BooleanSupplier demoScoring = () -> lastDemoScoring;

		/*
		 * operator console
		 */
		operatorConsole.button(1) // scissor in
			.and(demoScoring)
			.whileTrue(new MoveArm(arm, 0.2));
		operatorConsole.button(5) // scissor out
			.and(demoScoring)
			.whileTrue(new MoveArm(arm, -0.2));

		operatorConsole.button(6) // elevator up
			.and(demoScoring)
			.whileTrue(new MoveElevator(elevator, 0.2));
		operatorConsole.button(10) // elevator down
			.and(demoScoring)
			.whileTrue(new MoveElevator(elevator, -0.2));

		operatorConsole.button(2)
			.and(demoScoring)
			.whileTrue(new ReleaseGrabber(grabber));
		operatorConsole.button(3)
			.and(demoScoring)
			.whileTrue(new GrabGrabber(grabber, GrabberConstants.coneGrabSpeed));
		operatorConsole.button(4)
			.and(demoScoring)
			.whileTrue(new GrabGrabber(grabber, GrabberConstants.cubeGrabSpeed));

		// demoScoring will be killed by pid controller
		operatorConsole.button(14) // double substation
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.DoubleSubstation));
		operatorConsole.button(7) // high cone
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.HighCone));
		operatorConsole.button(8) // high cube
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.HighCube));
		operatorConsole.button(11) // mid cone
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.MidCone));
		operatorConsole.button(12) // mid cube
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.MidCube));
		operatorConsole.button(15) // all in
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.AllIn));
		operatorConsole.button(16) // ground
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.Ground));

		/** @link {Robot#robotPeriodic} */
		// making this not a command as it kills all commands
		// operatorConsole.button(13)
		// .whileTrue(Commands.runEnd(this::killEverything, () -> {
		// killed = false;
		// }));
	}

	public boolean killed = false;

	public void killEverything() {
		killed = true;
		CommandScheduler.getInstance().cancelAll();
		swerve.setChasisSpeeds(new ChassisSpeeds(0, 0, 0));
		elevator.stop(); // killMotor();
		arm.stop();
		grabber.stop();
	}

	// @SuppressWarnings("unused")
	private void configureCameras() {
		// if (!Robot.isReal())
		// return;

		// camera
		// try {
		// var camera = CameraServer.startAutomaticCapture("arm", 0);
		// camera.setConnectVerbose(0);
		// // camera.setFPS(CameraConstants.fps);
		// // camera.setResolution(CameraConstants.width, CameraConstants.height);
		// // camera.setPixelFormat(PixelFormat.kGray);
		// if (CameraConstants.exposure >= 0)
		// camera.setExposureManual(CameraConstants.exposure);
		// else
		// camera.setExposureAuto();
		// // camera.setBrightness(0);

		// // camera server is evil
		// // var cameraServer = CameraServer.addSwitchedCamera("camera");
		// // var cameraServer = CameraServer.startAutomaticCapture(camera);
		// // var cameraServer = (MjpegServer) CameraServer.getServer();
		// // cameraServer.setFPS(CameraConstants.fps);
		// // cameraServer.setResolution(CameraConstants.width, CameraConstants.height);
		// // cameraServer.setCompression(CameraConstants.compression);

		// Shuffleboard.getTab(ShuffleboardConstants.driveTab)
		// .add("camera", SendableCameraWrapper.wrap(camera))
		// // .addCamera("camera", "arm", cameraServ
		// .withWidget(BuiltInWidgets.kCameraStream)
		// .withSize(8, 6)
		// .withPosition(0, 0);
		// } catch (edu.wpi.first.cscore.VideoException e) {
		// DriverStation.reportError("Failed to get camera: " + e.toString(), e.getStackTrace());
		// }

		// var limelightCamera = new HttpCamera("limelight-gpv-stream", "http://10.1.2.12:5800",
		// HttpCameraKind.kMJPGStreamer);

		Shuffleboard.getTab(ShuffleboardConstants.driveTab)
			.add("gpv", SendableCameraWrapper.wrap("limelight-gpv-stream", "http://10.1.2.12:5800/stream.mjpg"))
			.withProperties(Map.of("show crosshair", false, "show controls", false))
			.withWidget(BuiltInWidgets.kCameraStream)
			.withSize(11, 5)
			.withPosition(0, 0);

		Shuffleboard.getTab(ShuffleboardConstants.driveTab)
			.add("fv", SendableCameraWrapper.wrap("limelight-fv-stream", "http://10.1.2.11:5800/stream.mjpg"))
			.withProperties(Map.of("show crosshair", false, "show controls", false))
			.withWidget(BuiltInWidgets.kCameraStream)
			.withSize(5, 5)
			.withPosition(11, 0);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		if (demoModeDash.getBoolean(lastDemoMode))
			return Commands.none();
		// if (autoDelay.getDouble(0) > 0)
		// return Commands.waitSeconds(autoDelay.getDouble(0)).andThen(autoChooser.get());
		return autoChooser.get().get(); // 😭
	}

	Alert driverControllerAlert = new Alert("driver controller not connected properly", AlertType.Error);
	Alert operatorConsoleAlert = new Alert("operator console not connected properly", AlertType.Error);
	Alert operatorJoystickAlert = new Alert("operator joystick not connected properly", AlertType.Error);

	public void updateOIAlert() {
		if (!driverController.getHID().isConnected()
			|| driverController.getHID().getName().indexOf("Xbox") < 0)
			driverControllerAlert.set(true);
		else
			driverControllerAlert.set(false);

		if (!operatorConsole.getHID().isConnected()
			|| operatorConsole.getHID().getName().indexOf("CyController") < 0)
			operatorConsoleAlert.set(true);
		else
			operatorConsoleAlert.set(false);

		if (!operatorJoystick.getHID().isConnected()
			|| operatorJoystick.getHID().getName().indexOf("Logitech") < 0)
			operatorJoystickAlert.set(true);
		else
			operatorJoystickAlert.set(false);
	}
}
