package overture.sim.robots;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class SimBaseRobot {
    private String name;

    public SimBaseRobot(String name, Pose2d startingPose) {
        this.name = name;
    }

    public String GetName() {
        return name;
    }

    public abstract void Update();

    public abstract AbstractDriveTrainSimulation GetDriveTrain();
}
