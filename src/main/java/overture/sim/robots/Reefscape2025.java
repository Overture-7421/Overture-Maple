package overture.sim.robots;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import overture.sim.elevator.Elevator;
import overture.sim.swerve.Constants;
import overture.sim.swerve.SwerveChassis;

public class Reefscape2025 extends SimBaseRobot {
    SwerveChassis driveTrain;
    Elevator elevator;

    public Reefscape2025(String name, Pose2d startingPose) {
        super(name, startingPose);

        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());
        elevator = new Elevator(this, 
                                new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), new Rotation3d()), 
                                "elevator", 
                                DCMotor.getFalcon500(1),
                                30.0,
                                Kilograms.of(0.5),
                                Meters.of(0.1),
                                Meters.of(0.0),
                                Meters.of(1.0),
                                Meters.of(0.0),
                                false);
    }

    @Override
    public void Update() {
        driveTrain.Update();
        elevator.Update();
    }

    @Override
    public AbstractDriveTrainSimulation GetDriveTrain() {
        return driveTrain;
    }
    
}
