package com.team254.lib.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.Supplier;

public class SimTalonFXWithCancoder extends SimTalonFXIO {
    ServoMotorSubsystemWithCanCoderConfig canCoderConfig;

    public SimTalonFXWithCancoder(ServoMotorSubsystemWithCanCoderConfig config) {
        super(
                config,
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60Foc(1),
                                config.momentOfInertia,
                                1.0 / config.ratioForSim),
                        DCMotor.getKrakenX60Foc(1),
                        0.001,
                        0.001));
        this.canCoderConfig = config;
    }

    @Override
    public Supplier<SimCanCoderIO.SimCanCoderState> getSupplierForCancoder(
            ServoMotorSubsystemWithCanCoderConfig c) {
        return () -> {
            var a = new SimCanCoderIO.SimCanCoderState();
            double ratio =
                    this.canCoderConfig.ratioForSim / this.canCoderConfig.cancoderUnitsForSim;
            a.positionRotations = lastRotations.get() * ratio;
            a.velocityRotations = lastRPS.get() * ratio;
            return a;
        };
    }

    @Override
    protected double getSimRatio() {
        return canCoderConfig.ratioForSim;
    }
}
