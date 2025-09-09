package com.team254.frc2025.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    double getRotationY();

    Trigger resetGyro();
}
