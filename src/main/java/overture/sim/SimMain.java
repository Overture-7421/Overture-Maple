package overture.sim;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.littletonrobotics.junction.Logger;
import overture.sim.swerve.SwerveChassis;

public class SimMain {
    SimulatedArena arena = null;
    SwerveChassis chassis = null;

    public void Initialize() {
        // Mechanisms
        chassis = new SwerveChassis("Offseason 2024", new Pose2d(3, 3, new Rotation2d()));

        // Arena
        SimulatedArena.overrideInstance(new Arena2024Crescendo());
        arena = SimulatedArena.getInstance();

        // Add game pieces to Arena
        arena.addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));

        // Add mechanisms to Arena
        arena.addDriveTrainSimulation(chassis);
    }


    public void Periodic() {
        arena.simulationPeriodic();
        chassis.Update();

        Logger.recordOutput("FieldSimulation/RobotPosition", chassis.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Notes", arena.getGamePiecesArrayByType("Note"));
    }
}
