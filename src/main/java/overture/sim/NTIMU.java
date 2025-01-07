package overture.sim;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;

import java.util.function.Supplier;

public class NTIMU {
    public static class Config {
        public String Name;

        public Supplier<Angle> Roll, Pitch, Yaw;
    }

    public DoublePublisher rollEntry, pitchEntry, yawEntry;
    private Config config;

    public NTIMU(Config config) {
        this.config = config;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        NetworkTable motorTable = ntInst.getTable(config.Name);

        rollEntry = motorTable.getDoubleTopic("roll").publish();
        pitchEntry = motorTable.getDoubleTopic("pitch").publish();
        yawEntry = motorTable.getDoubleTopic("yaw").publish();
    }

    public void Update() {
        rollEntry.set(config.Roll.get().in(Degrees));
        pitchEntry.set(config.Pitch.get().in(Degrees));
        yawEntry.set(config.Yaw.get().in(Degrees));
    }
}
