package overture.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class NTMotor {
    public static class Config {
        public String Name;

        public Consumer<Voltage> VoltageApplied;
        public Supplier<Angle> EncoderPosition;
        public Supplier<AngularVelocity> EncoderSpeed;
        public Supplier<Current> Current;
        public Boolean Inverted;
    }

    private DoublePublisher encoderSpeedEntry, encoderPositionEntry, currentEntry, torqueAppliedEntry;
    private DoubleSubscriber voltageEntry;
    private Config config;
    private Double invMultiplier = 1.0;

    public NTMotor(Config config) {
        this.config = config;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        NetworkTable motorTable = ntInst.getTable(config.Name);

        encoderSpeedEntry = motorTable.getDoubleTopic("encoder_speed").publish();
        encoderPositionEntry = motorTable.getDoubleTopic("encoder_position").publish();
        currentEntry = motorTable.getDoubleTopic("current").publish();
        torqueAppliedEntry = motorTable.getDoubleTopic("torque").publish();

        voltageEntry = motorTable.getDoubleTopic("voltage_applied").subscribe(0);

        motorTable.getEntry("voltage_applied").setDouble(0.0);

        encoderSpeedEntry.set(0);
        encoderPositionEntry.set(0);
        currentEntry.set(0);
        torqueAppliedEntry.set(0);

        if (config.Inverted) {
            invMultiplier = -1.0;
        }
    }

    public void Update() {
        config.VoltageApplied.accept(Volts.of(voltageEntry.get()).times(invMultiplier));

        encoderPositionEntry.set(config.EncoderPosition.get().in(Rotations) * invMultiplier);
        encoderSpeedEntry.set(config.EncoderSpeed.get().in(RotationsPerSecond) * invMultiplier);
        currentEntry.set(config.Current.get().in(Amps));
    }
}
