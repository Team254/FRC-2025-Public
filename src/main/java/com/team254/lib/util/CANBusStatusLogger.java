package com.team254.lib.util;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class CANBusStatusLogger {
    private final CANBus bus;

    public CANBusStatusLogger(String name) {
        bus = new CANBus(name);
    }

    public void logStatus() {
        var status = bus.getStatus();
        Logger.recordOutput("CANBusStatus/" + bus.getName(), status.Status);
        SmartDashboard.putString("CANBusStatus/" + bus.getName(), status.Status.getName());
    }
}
