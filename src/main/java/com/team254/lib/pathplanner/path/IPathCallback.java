package com.team254.lib.pathplanner.path;

import java.util.List;

// Interface for a callback that gets invoked after path creation but before trajectory generation,
// designed for callers to specify constraint zones, markers, point towards zones, and rotation
// targets etc.
// with the benefit of seeing the geometry of the path first.
public interface IPathCallback {
    public class PathMarkup {
        public List<ConstraintsZone> constraintsZones;
        public List<EventMarker> eventMarkers;
    }

    // Returns true if markup has changed since the last call to apply().
    // Gets checked every loop during path following to re-call apply() if necessary.
    boolean markupUpdated();

    public PathMarkup apply(PathPlannerPath path);
}
