package overture.sim.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Timer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import overture.sim.NTMotor;
import overture.sim.robots.SimBaseRobot;

public class Elevator {
  private SimBaseRobot robot;
  private NTMotor motor;
  private Transform3d robotToElevator;
  private ElevatorSim elevatorSim;
  private double lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

  public Elevator(SimBaseRobot robot,
      Transform3d robotToElevator,
      String name,
      DCMotor gearbox,
      double gearing,
      Mass carriageMassKg,
      Distance drumRadiusMeters,
      Distance minHeightMeters,
      Distance maxHeightMeters,
      Distance startingHeightMeters,
      boolean inverted) {

    elevatorSim = new ElevatorSim(gearbox, gearing, carriageMassKg.in(Kilograms), drumRadiusMeters.in(Meters),
        minHeightMeters.in(Meters), maxHeightMeters.in(Meters), true, startingHeightMeters.in(Meters));

    this.robot = robot;
    double metersToEncoderFactor = 1.0 / (drumRadiusMeters.in(Meters) * 2.0 * Math.PI) * gearing;
    this.robotToElevator = robotToElevator;

    motor = new NTMotor(new NTMotor.Config() {
      {
        Name = robot.GetName() + "/motors/" + name;
        VoltageApplied = (voltage) -> elevatorSim.setInputVoltage(voltage.in(Volts));
        EncoderPosition = () -> Revolutions.of(elevatorSim.getPositionMeters() * metersToEncoderFactor);
        EncoderSpeed = () -> RevolutionsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() * metersToEncoderFactor);
        Current = () -> Amps.of(elevatorSim.getCurrentDrawAmps());
        Inverted = inverted;
      }
    });
  }

  public void Update() {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    elevatorSim.update(currentTime - lastTime);
    motor.Update();
    Logger.recordOutput(robot.GetName() + "/SimulatedElevator/" + robot.GetName() + "/Pose", GetEndEffectorPose());
    lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  public void SetRobotToElevator(Transform3d robotToElevator) {
    this.robotToElevator = robotToElevator;
  }

  private Pose3d GetEndEffectorPose() {
    return new Pose3d(robot.GetDriveTrain().getSimulatedDriveTrainPose()).transformBy(robotToElevator)
        .transformBy(new Transform3d(Meters.of(0), Meters.of(0), Meters.of(elevatorSim.getPositionMeters()), new Rotation3d()));
  }
}