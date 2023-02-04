package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ChargeStationBalance;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/**
	 * Subsystems
	 */
	public final Swerve swerve = new Swerve(gyro);
	public final Vision vision = new Vision();

	public final CommandXboxController driverController = new CommandXboxController(
		OperatorConstants.driverControllerPort);
	public final CommandXboxController operatorController = new CommandXboxController(
		OperatorConstants.operatorControllerPort);

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto mode");

	private AprilTagVision aprilTagVision = new AprilTagVision(vision, operatorController)

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer()
	{
		DriverStation.silenceJoystickConnectionWarning(true);

		// will be automatically scheduled when no other scheduled commands require swerve
		swerve.setDefaultCommand(new TeleopSwerve(swerve, driverController.getHID()));

		configureBindings();

		// setup autos
		autoChooser.addDefaultOption("Nothing", new InstantCommand());
		autoChooser.addDefaultOption("PP test", Autos.pathPlannerTest(swerve));

		SmartDashboard.putData("do balancing", new ChargeStationBalance(swerve));
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController Xbox} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		/* driver */
		driverController.a().onTrue(new InstantCommand(() -> swerve.toggleFieldRelative()));
		driverController.b().onTrue(new InstantCommand(() -> swerve.zeroYaw()));

		/* operator */
		operatorController.povLeft().and(operatorController.x())
			.whileTrue(new AprilTagVision(vision, AprilTagVision.Routine.LeftGrid));
		operatorController.povLeft().and(operatorController.a())
			.whileTrue(new AprilTagVision(vision, AprilTagVision.Routine.MiddleGrid));
		operatorController.povLeft().and(operatorController.b())
			.whileTrue(new AprilTagVision(vision, AprilTagVision.Routine.RightGrid));
		// operatorController.povUp().whileTrue();
		// operatorController.povRight().whileTrue();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
