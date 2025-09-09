package com.team254.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.CANDeviceId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class CANStatusLogger {
    private static CANStatusLogger instance;
    private final List<DeviceStatusInfo> devices = new ArrayList<>();

    private static class DeviceStatusInfo {
        private final String name;
        private final TalonFX talon;
        private final int deviceId;
        private final String canBus;
        private final StatusSignal<?> supplyVoltage;

        public DeviceStatusInfo(String name, TalonFX talon, int deviceId, String canBus) {
            this.name = name;
            this.talon = talon;
            this.deviceId = deviceId;
            this.canBus = canBus;
            this.supplyVoltage = talon.getSupplyVoltage();

            if (this.supplyVoltage != null) {
                this.supplyVoltage.setUpdateFrequency(100);
            }
        }
    }

    private CANStatusLogger() {}

    public static synchronized CANStatusLogger getInstance() {
        if (instance == null) {
            instance = new CANStatusLogger();
        }
        return instance;
    }

    public void registerTalonFX(String name, TalonFX talon, int deviceId, String canBus) {
        devices.add(new DeviceStatusInfo(name, talon, deviceId, canBus));
    }

    public void registerTalonFX(String name, TalonFX talon, CANDeviceId deviceId) {
        registerTalonFX(name, talon, deviceId.getDeviceNumber(), deviceId.getBus());
    }

    public void registerCANcoder(String name, CANcoder cancoder, int deviceId, String canBus) {
        // This method is kept for compatibility but doesn't need to track CANcoders
    }

    private BaseStatusSignal[] signals;

    private void initializeSignalArray() {
        int validSignalCount = 0;
        for (DeviceStatusInfo device : devices) {
            if (device.supplyVoltage != null) {
                validSignalCount++;
            }
        }

        if (signals == null || signals.length != validSignalCount) {
            signals = new BaseStatusSignal[validSignalCount];
            int index = 0;
            for (DeviceStatusInfo device : devices) {
                if (device.supplyVoltage != null) {
                    signals[index++] = device.supplyVoltage;
                }
            }
        }
    }

    public void updateCanStatus() {
        if (signals == null) {
            initializeSignalArray();
        }
        try {
            BaseStatusSignal.refreshAll(signals);
        } catch (Exception e) {
        }

        // Check each device status
        for (int i = 0; i < devices.size(); i++) {
            DeviceStatusInfo device = devices.get(i);
            String deviceName = device.name + "ID" + device.deviceId;
            boolean isConnected = false;

            if (device.talon != null && device.supplyVoltage != null) {
                isConnected = (device.supplyVoltage.getStatus() == StatusCode.OK);
            }

            Logger.recordOutput("CANstatus/" + deviceName, isConnected);
            SmartDashboard.putBoolean("CAN/" + device.canBus + "/" + deviceName, isConnected);
        }
    }
}
