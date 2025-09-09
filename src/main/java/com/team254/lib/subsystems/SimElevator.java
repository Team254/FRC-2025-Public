package com.team254.lib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class SimElevator {
    public static class SimElevatorConfig {
        public double gearing;
        public double carriageMass; // in KG
        public double drumRadius;

        // This is rotor to the height of the elevator.
        // rotor * this number = height of elevator.
        public double meterToRotorRatio;
    }

    protected static class SimElevatorTalonFX extends TalonFXIO {
        TalonFXSimState simState;

        public SimElevatorTalonFX(ServoMotorSubsystemConfig config) {
            super(config);
            talon.getSimState().Orientation =
                    (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;
            simState = talon.getSimState();
        }
    }

    protected ServoMotorSubsystemWithFollowersConfig config;
    protected SimElevatorConfig simConfig;
    protected SimElevatorTalonFX leadIO;
    protected SimElevatorTalonFX[] followerIO;

    protected ElevatorSim sim;
    protected Notifier simNotifier = null;
    protected double lastUpdateTimestamp = 0.0;

    public TalonFXIO getLeadIO() {
        return leadIO;
    }

    public TalonFXIO[] getFollowerIO() {
        return followerIO;
    }

    public SimElevator(ServoMotorSubsystemWithFollowersConfig config, SimElevatorConfig simConfig) {
        this.config = config;
        this.simConfig = simConfig;
        sim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(config.followers.length + 1),
                        1.0 / simConfig.gearing,
                        simConfig.carriageMass,
                        simConfig.drumRadius,
                        config.kMinPositionUnits,
                        config.kMaxPositionUnits,
                        true,
                        0.0);

        leadIO = new SimElevatorTalonFX(config);
        followerIO =
                Arrays.stream(config.followers)
                        .map(followerConfig -> new SimElevatorTalonFX(followerConfig.config))
                        .toList()
                        .toArray(new SimElevatorTalonFX[0]);

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier =
                new Notifier(
                        () -> {
                            updateSimState();
                        });
        simNotifier.startPeriodic(0.005);
    }

    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

    protected void updateSimState() {
        var simState = leadIO.simState;
        double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

        sim.setInput(simVoltage);
        Logger.recordOutput(config.name + "/Sim/SimulatorVoltage", simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        sim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        // Find current state of sim in M
        double simPositionM = sim.getPositionMeters();
        Logger.recordOutput(config.name + "/Sim/SimulatorPositionMeters", simPositionM);

        // Mutate rotor position
        double rotorPosition = simPositionM / simConfig.meterToRotorRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput(config.name + "/Sim/setRawRotorPosition", rotorPosition);

        // Mutate rotor vel
        double rotorVel = sim.getVelocityMetersPerSecond() / simConfig.meterToRotorRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput(
                config.name + "/Sim/SimulatorVelocityMS", sim.getVelocityMetersPerSecond());

        for (int i = 0; i < followerIO.length; ++i) {
            followerIO[i].simState.setRawRotorPosition(
                    rotorPosition * (this.config.followers[i].inverted ? 1.0 : -1.0));
            followerIO[i].simState.setRotorVelocity(
                    rotorVel * (this.config.followers[i].inverted ? 1.0 : -1.0));
        }
    }
}
