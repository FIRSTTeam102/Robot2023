package frc.robot.commands;

import frc.robot.util.PolynomialRegression;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * @see https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/commands/FeedForwardCharacterization.java
 */
public class FeedForwardCharacterization extends CommandBase {
	private static final double startDelaySecs = 2.0;
	private static final double rampRateVoltsPerSec = 0.05;

	private final boolean forwards;
	private final boolean isDrive;

	private final FeedForwardCharacterizationData dataPrimary;
	private final FeedForwardCharacterizationData dataSecondary;
	private final Consumer<Double> voltageConsumerSimple;
	private final BiConsumer<Double, Double> voltageConsumerDrive;
	private final Supplier<Double> velocitySupplierPrimary;
	private final Supplier<Double> velocitySupplierSecondary;

	private final Timer timer = new Timer();

	/** for a drive */
	public FeedForwardCharacterization(SubsystemBase drive, boolean forwards,
		BiConsumer<Double, Double> voltageConsumer,
		Supplier<Double> leftVelocitySupplier,
		Supplier<Double> rightVelocitySupplier) {
		addRequirements(drive);
		this.forwards = forwards;
		this.isDrive = true;
		this.dataPrimary = new FeedForwardCharacterizationData(drive.getName() + "Left");
		this.dataSecondary = new FeedForwardCharacterizationData(drive.getName() + "Right");
		this.voltageConsumerSimple = null;
		this.voltageConsumerDrive = voltageConsumer;
		this.velocitySupplierPrimary = leftVelocitySupplier;
		this.velocitySupplierSecondary = rightVelocitySupplier;
	}

	/** for a simple subsystem */
	public FeedForwardCharacterization(SubsystemBase subsystem, boolean forwards,
		Consumer<Double> voltageConsumer,
		Supplier<Double> velocitySupplier) {
		addRequirements(subsystem);
		this.forwards = forwards;
		this.isDrive = false;
		this.dataPrimary = new FeedForwardCharacterizationData(subsystem.getName());
		this.dataSecondary = null;
		this.voltageConsumerSimple = voltageConsumer;
		this.voltageConsumerDrive = null;
		this.velocitySupplierPrimary = velocitySupplier;
		this.velocitySupplierSecondary = null;
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		if (timer.get() < startDelaySecs) {
			if (isDrive) {
				voltageConsumerDrive.accept(0.0, 0.0);
			} else {
				voltageConsumerSimple.accept(0.0);
			}
		} else {
			double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (forwards ? 1 : -1);
			if (isDrive) {
				voltageConsumerDrive.accept(voltage, voltage);
			} else {
				voltageConsumerSimple.accept(voltage);
			}
			dataPrimary.add(velocitySupplierPrimary.get(), voltage);
			if (isDrive) {
				dataSecondary.add(velocitySupplierSecondary.get(), voltage);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (isDrive) {
			voltageConsumerDrive.accept(0.0, 0.0);
		} else {
			voltageConsumerSimple.accept(0.0);
		}
		timer.stop();
		dataPrimary.print();
		if (isDrive) {
			dataSecondary.print();
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public static class FeedForwardCharacterizationData {
		private final String name;
		private final List<Double> velocityData = new LinkedList<>();
		private final List<Double> voltageData = new LinkedList<>();

		public FeedForwardCharacterizationData(String name) {
			this.name = name;
		}

		public void add(double velocity, double voltage) {
			if (Math.abs(velocity) > 1E-4) {
				velocityData.add(Math.abs(velocity));
				voltageData.add(Math.abs(voltage));
			}
		}

		public void print() {
			PolynomialRegression regression = new PolynomialRegression(
				velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
				voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
				1);

			System.out.println("FF Characterization Results (" + name + "):");
			System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
			System.out.println(String.format("\tR2=%.5f", regression.R2()));
			System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
			System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
		}
	}
}