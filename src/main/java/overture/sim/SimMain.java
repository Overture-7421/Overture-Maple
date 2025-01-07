package overture.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;
import overture.sim.swerve.SwerveChassis;

public class SimMain {
	SimulatedArena arena = null;
	SwerveChassis chassis = null;

	public void Initialize() {
		// Mechanisms
		chassis = new SwerveChassis("Offseason 2024", new Pose2d(3, 3, new Rotation2d()));

		// Arena
		SimulatedArena.overrideInstance(new Arena2025Reefscape());
		arena = SimulatedArena.getInstance();

		// Add game pieces to Arena
		// arena.addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(3, 3)));

		arena.resetFieldForAuto();

		// Add mechanisms to Arena
		arena.addDriveTrainSimulation(chassis);
	}

	public void Periodic() {
		arena.simulationPeriodic();
		chassis.Update();

		Logger.recordOutput("FieldSimulation/RobotPosition", chassis.getSimulatedDriveTrainPose());
		Logger.recordOutput("FieldSimulation/Algae",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		Logger.recordOutput("FieldSimulation/Coral",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
	}
}
