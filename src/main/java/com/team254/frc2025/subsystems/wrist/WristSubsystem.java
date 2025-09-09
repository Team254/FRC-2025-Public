package com.team254.frc2025.subsystems.wrist;

import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.subsystems.superstructure.SuperstructureState;
import com.team254.lib.subsystems.*;
import com.team254.lib.util.Util;
import edu.wpi.first.math.util.Units;

public class WristSubsystem
        extends ServoMotorSubsystemWithCanCoder<
                MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
    private RobotState state;

    public WristSubsystem(
            ServoMotorSubsystemWithCanCoderConfig c,
            MotorIO motorIO,
            CanCoderIO cancoderIO,
            RobotState state) {
        super(c, new MotorInputsAutoLogged(), motorIO, new CanCoderInputsAutoLogged(), cancoderIO);
        this.state = state;
        this.positionSetpointUnits = Constants.WristConstants.kWristStowCoralPositionRadians;
        setDefaultCommand(
                motionMagicSetpointCommand(this::getPositionSetpointUnits)
                        .withName("Wrist Maintain Setpoint (default)")
                        .ignoringDisable(true));

        // Update frequency for feedback.
        cancoderIO.updateFrequency(500);
    }

    // Updates robot state with current wrist angle
    @Override
    public void periodic() {
        super.periodic();
        state.setWristRadians(inputs.unitPosition);
    }

    public boolean isStowed() {
        // Returns true if wrist is in stowed position
        return getCurrentPosition() < Constants.WristConstants.kWristToleranceRadians;
    }

    // Returns true if wrist is at the specified superstructure level
    public boolean isAtLevel(SuperstructureState state) {
        return Util.epsilonEquals(
                getCurrentPosition(),
                state.getWristAngleRad(),
                Constants.WristConstants.kWristToleranceRadians);
    }

    public boolean clearOfIndexer() {
        // Returns true if wrist clears the indexer mechanism
        return getCurrentPosition() > Units.degreesToRadians(-75);
    }
}
