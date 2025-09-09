package com.team254.frc2025.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code IndexerSensorIO} interface defines the input/output operations for indexer sensors. It
 * provides methods to read sensor inputs such as banner sensor states for detecting game pieces.
 */
public interface IndexerSensorIO {
    @AutoLog
    public class IndexerSensorInputs {
        public boolean firstIndexerBannerTriggered;
        public boolean secondIndexerBannerTriggered;
    }

    public default void readInputs(IndexerSensorIO.IndexerSensorInputs inputs) {}
}
