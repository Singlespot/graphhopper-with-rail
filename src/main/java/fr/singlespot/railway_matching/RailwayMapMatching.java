package fr.singlespot.railway_matching;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.*;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class RailwayMapMatching extends MapMatching {

    public RailwayMapMatching(BaseGraph graph, LocationIndexTree locationIndex, Router router) {
        super(graph, locationIndex, router);
    }

    public static RailwayMapMatching fromGraphHopper(GraphHopper graphHopper, PMap hints) {
        Router router = routerFromGraphHopper(graphHopper, hints);
        return new RailwayMapMatching(graphHopper.getBaseGraph(), (LocationIndexTree) graphHopper.getLocationIndex(), router);
    }

    /**
     * This method does the actual map matching.
     * <p>
     * It will throw an exception if a segment of the input list cannot be matched.
     *
     * @param observations the input list with GPX points which should match to edges
     *                     of the graph specified in the constructor
     * @param sw           The stopwatch
     * @param routedPaths  The list of routed path between the first and last observation
     */
    public MatchResult match_with_routing(List<Observation> observations, StopWatch sw, List<Path> routedPaths,
                                          boolean forceInitialRouting) {
        return match_with_routing(observations, false, 0, sw, routedPaths, forceInitialRouting);
    }

    /**
     * This method does the actual map matching.
     * <p>
     * It will start at the provided index.
     *
     * @param observations The input list with GPX points which should match to edges
     *                     of the graph specified in the constructor
     * @param ignoreErrors Whether to ignore unmatchable segments.
     * @param offset       Offset to start matching at. This value will be stored and available
     *                     using getSuccessfullyMatchedPoints().
     * @param sw           The stopwatch
     * @param routedPaths  The list of routed path between the first and last observation
     */
    public MatchResult match_with_routing(List<Observation> observations, boolean ignoreErrors, int offset, StopWatch sw,
                                          List<Path> routedPaths, boolean forceInitialRouting) {
        this.offset = offset;
        boolean usedDirectRouting = false;
        boolean forcedDirectRouting = false;
        resetCounters(observations.size(), offset);
        List<Observation> observationSubList = observations.subList(offset, observations.size());
        List<Observation> filteredObservations = filterObservations(observationSubList);
        statistics.put("filteredObservations", filteredObservations.size());

        // Snap observations to links. Generates multiple candidate snaps per observation.
        java.util.function.Function<Observation, List<Snap>> findCandidateSnaps = o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, o.getPoint().accuracy, o.getPoint().index, o.getPoint().timestamp);
        List<List<Snap>> snapsPerObservationTmp = filteredObservations.stream()
                .map(findCandidateSnaps)
                .collect(Collectors.toList());
        queryGraph = QueryGraph.create(graph, snapsPerObservationTmp.stream().flatMap(Collection::stream).collect(Collectors.toList()));

        MatchResult result;
        List<SequenceState<State, Observation, Path>> seq;
        Path routedPath = null;
        boolean anySnapNotOnAnyRoutedPath = false;
        List<List<Snap>> snapsPerObservationOnRoutedPath = new ArrayList<>();
// Check if there is at least one valid routed path
        if (routedPaths.get(0) != null && routedPaths.stream().anyMatch(Path::isFound)) {
            SnappedPointsAnalyzer analyzer = new SnappedPointsAnalyzer();
            SnappedPointsAnalyzer.AnalysisResult analysisResult = analyzer.analyze(
                    routedPaths,
                    filteredObservations,
                    findCandidateSnaps,
                    this::resolveToRealEdge
            );
            int[] snapsOnPathCounts = analysisResult.snapsOnPathCounts;
            List<List<Boolean>> snapsNotOnRoutedPaths = analysisResult.snapsNotOnRoutedPaths;
            List<Set<Integer>> routedPathsPathEdgeIndices = analysisResult.routedPathsPathEdgeIndices;
            List<List<List<Snap>>> snapsPerObservationOnRoutedPathTmpList = analysisResult.snapsPerObservationOnRoutedPathTmpList;

            // Variables to track the path with the most snaps
            int maxSnapsCount = -1;
            int bestPathIndex = -1;
            Path bestPath = null;
            List<List<Snap>> bestPathSnaps = new ArrayList<>();

            // Loop over all routed paths to find the routed path that matches all observations
            for (int routedPathsIndex = 0, routedPathsSize = routedPaths.size(); routedPathsIndex < routedPathsSize; routedPathsIndex++) {
                // Get the current routed path to check
                Path tmpRoutedPath = routedPaths.get(routedPathsIndex);
                // Initialize a list to store snaps that are on this routed path
                List<List<Snap>> snapsPerObservationOnRoutedPathTmp = new ArrayList<>();
                // Get the set to store edge indices for this path
                Set<Integer> pathEdgeIndices = routedPathsPathEdgeIndices.get(routedPathsIndex);
                // Skip invalid paths
                if (!tmpRoutedPath.isFound()) {
                    continue;
                }
                // Get all edges of the path
                List<EdgeIteratorState> pathEdges = tmpRoutedPath.calcEdges();
                // Track the maximum edge index found for ordering
                int maxEdgeIndex = -1;

                // Counter for snaps on this path
                int snapsOnPathCount = snapsOnPathCounts[routedPathsIndex];

                // Debug output about the path
                System.out.println("Path #" + (routedPathsIndex + 1) + ": " + snapsOnPathCount + " snaps out of " +
                        snapsPerObservationTmp.size() + ", edges used: " + pathEdgeIndices.size());

                // Debug output if all snaps are on the first and last edges only
                if (pathEdgeIndices.size() <= 2)
                    System.out.println("Path #" + (routedPathsIndex + 1) + ", all snaps on the first and last edges");

                // Store current path index for use in lambda
                int finalRoutedPathsIndex = routedPathsIndex;
                // Check if all snaps are on this routed path
                boolean allSnapsOnRoutedPath = snapsNotOnRoutedPaths.stream().noneMatch(snap -> snap.get(finalRoutedPathsIndex));

                // Track the path with the most snaps (used for both forcing and via-waypoint routing)
                if (snapsOnPathCount > maxSnapsCount &&
                        (pathEdgeIndices.size() > 2 || filteredObservations.size() == 2)) {
                    maxSnapsCount = snapsOnPathCount;
                    bestPathIndex = routedPathsIndex;
                    bestPath = tmpRoutedPath;
                    bestPathSnaps = new ArrayList<>(snapsPerObservationOnRoutedPathTmpList.get(routedPathsIndex));
                }

                // If not all snaps are on the path but forcing is enabled, set forced routing flag
                if (!allSnapsOnRoutedPath && forceInitialRouting) {
                    forcedDirectRouting = true;
                }

                // Use this path if all snaps are on it (and not forcing)
                // AND either there are more than 2 edge indices or exactly 2 observations
                if (allSnapsOnRoutedPath && !forceInitialRouting &&
                        (pathEdgeIndices.size() > 2 || filteredObservations.size() == 2)) {
                    routedPath = tmpRoutedPath;
                    snapsPerObservationOnRoutedPath.addAll(snapsPerObservationOnRoutedPathTmpList.get(routedPathsIndex));
                    System.out.println("All observations on the path #" + (finalRoutedPathsIndex + 1) + ": using direct routing for map matching");
                    
                    // Print selected path as GeoJSON for direct routing
                    if (tmpRoutedPath.isFound()) {
                        PointList pathPoints = tmpRoutedPath.calcPoints();
                        StringBuilder geoJson = new StringBuilder();
                        geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
                        for (int i = 0; i < pathPoints.size(); i++) {
                            if (i > 0) geoJson.append(",");
                            geoJson.append("[").append(pathPoints.getLon(i)).append(",").append(pathPoints.getLat(i)).append("]");
                        }
                        geoJson.append("]},\"properties\":{\"stroke\":\"#0000ff\",\"path_index\":")
                                .append(finalRoutedPathsIndex)
                                .append(",\"distance\":")
                                .append(tmpRoutedPath.getDistance())
                                .append(",\"snaps\":")
                                .append(snapsOnPathCount)
                                .append(",\"selected\":true,\"direct_routing\":true}}");
                        System.out.println("SelectedPath GeoJSON (direct): " + geoJson);
                    }
                    
                    usedDirectRouting = true;
                    break;
                }
            }

            // If forcing routing and we found a best path, use it
            if (forceInitialRouting && bestPath != null) {
                routedPath = bestPath;
                snapsPerObservationOnRoutedPath.addAll(bestPathSnaps);
                System.out.println("Forced routing - SELECTED path #" + (bestPathIndex + 1) + " with " + maxSnapsCount +
                        " snaps out of " + snapsPerObservationTmp.size() + " for map matching");

                // Print bestPath as GeoJSON
                if (bestPath.isFound()) {
                    PointList bestPathPoints = bestPath.calcPoints();
                    StringBuilder geoJson = new StringBuilder();
                    geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
                    for (int i = 0; i < bestPathPoints.size(); i++) {
                        if (i > 0) geoJson.append(",");
                        geoJson.append("[").append(bestPathPoints.getLon(i)).append(",").append(bestPathPoints.getLat(i)).append("]");
                    }
                    geoJson.append("]},\"properties\":{\"stroke\":\"#ff0000\",\"path_index\":")
                            .append(bestPathIndex)
                            .append(",\"distance\":")
                            .append(bestPath.getDistance())
                            .append(",\"snaps\":")
                            .append(maxSnapsCount)
                            .append(",\"selected\":true}}");
                    System.out.println("BestPath GeoJSON: " + geoJson);
                }

                // Print summary of all path snap counts for comparison
                System.out.println("Path snap counts summary:");
                for (int i = 0; i < snapsOnPathCounts.length; i++) {
                    if (routedPaths.get(i).isFound()) {
                        String marker = (i == bestPathIndex) ? " [SELECTED]" : "";
                        System.out.println("  Path #" +
                                (i + 1) + ": " + snapsOnPathCounts[i] + " snaps out of " +
                                snapsPerObservationTmp.size() + marker);
                    }
                }
            }
            statistics.put("usedDirectRouting", usedDirectRouting);
            statistics.put("forcedDirectRouting", forcedDirectRouting);
            List<Integer> observationsNotOnAnyPathIndices = new ArrayList<>();
            for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
                List<Boolean> snapNotOnRoutedPath = snapsNotOnRoutedPaths.get(observationsIndex);
                List<Snap> snaps = snapsPerObservationTmp.get(observationsIndex);
                if (snapNotOnRoutedPath.stream().allMatch(Boolean::booleanValue) && !snaps.isEmpty()) {
                    System.out.println("Observation not on any path: " + snapsPerObservationTmp.get(observationsIndex).get(0).getQueryPoint());
                    if (!forceInitialRouting) {
                        anySnapNotOnAnyRoutedPath = true;
                        observationsNotOnAnyPathIndices.add(observationsIndex);
                    }
                }
            }

            // When not forcing and some snaps are not on any path, try routing via the off-path points
            if (anySnapNotOnAnyRoutedPath && bestPath != null && !forceInitialRouting) {
                // Rebuild off-path indices relative to the BEST path (not "any path")
                // so that anchors are guaranteed to be on the best path
                List<Integer> observationsNotOnBestPathIndices = new ArrayList<>();
                for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
                    if (snapsNotOnRoutedPaths.get(observationsIndex).get(bestPathIndex)) {
                        observationsNotOnBestPathIndices.add(observationsIndex);
                    }
                }
                System.out.println("Attempting via-waypoint routing through " + observationsNotOnBestPathIndices.size() +
                        " off-path observations (relative to best path #" + (bestPathIndex + 1) + ")" +
                        " (was " + observationsNotOnAnyPathIndices.size() + " off all paths)");

                // Build set for quick lookup
                Set<Integer> offPathSet = new LinkedHashSet<>(observationsNotOnBestPathIndices);
                int totalObs = filteredObservations.size();

                // Identify contiguous off-path segments with their on-path anchors
                // Each segment is: [lastOnPathBefore, offPath1, offPath2, ..., firstOnPathAfter]
                List<List<Integer>> offPathSegments = new ArrayList<>();
                List<Integer> currentSegment = null;
                for (int idx : observationsNotOnBestPathIndices) {
                    if (currentSegment == null || idx != currentSegment.get(currentSegment.size() - 1) + 1) {
                        // Start a new segment
                        if (currentSegment != null) {
                            offPathSegments.add(currentSegment);
                        }
                        currentSegment = new ArrayList<>();
                    }
                    currentSegment.add(idx);
                }
                if (currentSegment != null) {
                    offPathSegments.add(currentSegment);
                }

                System.out.println("Via-waypoint routing: found " + offPathSegments.size() + " contiguous off-path segment(s)");
                for (int segNum = 0; segNum < offPathSegments.size(); segNum++) {
                    List<Integer> seg = offPathSegments.get(segNum);
                    int anchorBefore = seg.get(0) - 1;
                    int anchorAfter = seg.get(seg.size() - 1) + 1;
                    System.out.println("  Segment " + segNum + ": off-path obs " + filteredObservations.get(seg.get(0)).getPoint().index + "-" + filteredObservations.get(seg.get(seg.size() - 1)).getPoint().index +
                            " (anchor before: obs " + (anchorBefore >= 0 ? filteredObservations.get(anchorBefore).getPoint().index : "NONE") +
                            ", anchor after: obs " + (anchorAfter < totalObs ? filteredObservations.get(anchorAfter).getPoint().index : "NONE") + ")");
                }

                // Collect snaps for all waypoints across all segments (for building the QueryGraph)
                List<Snap> allSegmentSnaps = new ArrayList<>();
                // Build waypoint lists for each segment: [anchorBefore, offPath1, ..., offPathN, anchorAfter]
                List<List<Integer>> segmentWaypointIndices = new ArrayList<>();
                boolean allWaypointsHaveSnaps = true;

                for (int segNum = 0; segNum < offPathSegments.size(); segNum++) {
                    List<Integer> seg = offPathSegments.get(segNum);
                    List<Integer> waypoints = new ArrayList<>();

                    // Add anchor before (last on-path obs before segment)
                    int anchorBefore = seg.get(0) - 1;
                    if (anchorBefore >= 0) {
                        waypoints.add(anchorBefore);
                    } else {
                        // First observation is off-path, use it as its own start
                        System.out.println("  Segment " + segNum + ": no on-path anchor before, first obs is off-path");
                    }

                    // Add all off-path observations in this segment
                    waypoints.addAll(seg);

                    // Add anchor after (first on-path obs after segment)
                    int anchorAfter = seg.get(seg.size() - 1) + 1;
                    if (anchorAfter < totalObs) {
                        waypoints.add(anchorAfter);
                    } else {
                        // Last observation is off-path, use it as its own end
                        System.out.println("  Segment " + segNum + ": no on-path anchor after, last obs is off-path");
                    }

                    segmentWaypointIndices.add(waypoints);
                }

                // Snap all waypoint observations
                Map<Integer, Snap> waypointSnapMap = new LinkedHashMap<>();
                for (List<Integer> waypoints : segmentWaypointIndices) {
                    for (int obsIdx : waypoints) {
                        if (waypointSnapMap.containsKey(obsIdx)) continue;
                        Observation obs = filteredObservations.get(obsIdx);
                        List<Snap> candidateSnaps = findCandidateSnaps(obs.getPoint().lat, obs.getPoint().lon,
                                Math.min(obs.getPoint().accuracy, 300.0), obs.getPoint().index, obs.getPoint().timestamp);
                        if (candidateSnaps.isEmpty()) {
                            System.out.println("  Obs " + obs.getPoint().index + ": NO SNAPS at " +
                                    obs.getPoint().lat + "," + obs.getPoint().lon +
                                    " - aborting via-waypoint routing");
                            allWaypointsHaveSnaps = false;
                            break;
                        }
                        Snap closestSnap = candidateSnaps.get(0);
                        waypointSnapMap.put(obsIdx, closestSnap);
                        allSegmentSnaps.add(closestSnap);
                        System.out.println("  Obs " + obs.getPoint().index + (offPathSet.contains(obsIdx) ? " [OFF-PATH]" : " [ON-PATH anchor]") +
                                ": GPS=" + obs.getPoint().lat + "," + obs.getPoint().lon +
                                " -> snapped to node " + closestSnap.getClosestNode() +
                                " at " + closestSnap.getSnappedPoint().lat + "," + closestSnap.getSnappedPoint().lon +
                                " on edge " + closestSnap.getClosestEdge().getEdge() +
                                " (name=" + closestSnap.getClosestEdge().getName() + ")" +
                                " dist=" + String.format("%.1f", closestSnap.getQueryDistance()) + "m" +
                                " (" + candidateSnaps.size() + " candidates)");
                    }
                    if (!allWaypointsHaveSnaps) break;
                }

                if (allWaypointsHaveSnaps) {
                    // Build a query graph for all waypoint snaps
                    QueryGraph waypointQueryGraph = QueryGraph.create(graph, allSegmentSnaps);

                    // Route each segment and collect edges
                    List<EdgeIteratorState> allRoutedEdges = new ArrayList<>();
                    boolean allSegmentsRouted = true;
                    double totalRoutedDistance = 0;

                    for (int segNum = 0; segNum < segmentWaypointIndices.size(); segNum++) {
                        List<Integer> waypoints = segmentWaypointIndices.get(segNum);
                        StringBuilder wpDesc = new StringBuilder();
                        for (int wi = 0; wi < waypoints.size(); wi++) {
                            if (wi > 0) wpDesc.append(" -> ");
                            wpDesc.append(filteredObservations.get(waypoints.get(wi)).getPoint().index);
                        }
                        System.out.println("Via-waypoint routing segment " + segNum + ": routing through " +
                                waypoints.size() + " waypoints (obs indices: " + wpDesc + ")");

                        for (int wpIdx = 0; wpIdx < waypoints.size() - 1; wpIdx++) {
                            int fromObsIdx = waypoints.get(wpIdx);
                            int toObsIdx = waypoints.get(wpIdx + 1);
                            Snap fromSnap = waypointSnapMap.get(fromObsIdx);
                            Snap toSnap = waypointSnapMap.get(toObsIdx);
                            int fromNode = fromSnap.getClosestNode();
                            int toNode = toSnap.getClosestNode();

                            System.out.println("  Leg " + wpIdx + ": obs " + fromSnap.getQueryPoint().index +
                                    " (node " + fromNode +
                                    " at " + fromSnap.getSnappedPoint().lat + "," + fromSnap.getSnappedPoint().lon +
                                    ", edge " + fromSnap.getClosestEdge().getEdge() +
                                    " '" + fromSnap.getClosestEdge().getName() + "')" +
                                    " -> obs " + toSnap.getQueryPoint().index +
                                    " (node " + toNode +
                                    " at " + toSnap.getSnappedPoint().lat + "," + toSnap.getSnappedPoint().lon +
                                    ", edge " + toSnap.getClosestEdge().getEdge() +
                                    " '" + toSnap.getClosestEdge().getName() + "')");

                            List<Path> legPaths = router.calcPaths(waypointQueryGraph, fromNode, toNode,
                                    new int[]{fromNode}, new int[]{toNode});
                            if (legPaths.isEmpty() || !legPaths.get(0).isFound()) {
                                System.out.println("    -> FAILED: no path found");
                                allSegmentsRouted = false;
                                break;
                            }
                            Path legPath = legPaths.get(0);
                            List<EdgeIteratorState> legEdges = legPath.calcEdges();
                            allRoutedEdges.addAll(legEdges);
                            totalRoutedDistance += legPath.getDistance();
                            System.out.println("    -> OK: " + legEdges.size() + " edges, distance=" +
                                    String.format("%.1f", legPath.getDistance()) + "m, time=" + legPath.getTime() + "ms");
                        }
                        if (!allSegmentsRouted) break;
                    }

                    if (allSegmentsRouted && !allRoutedEdges.isEmpty()) {
                        // Merge: best path edges + routed segment edges
                        Set<Integer> mergedEdgeIds = new LinkedHashSet<>();

                        // Add best path edges
                        List<EdgeIteratorState> bestPathEdges = bestPath.calcEdges();
                        for (EdgeIteratorState e : bestPathEdges) {
                            mergedEdgeIds.add(resolveToRealEdge(e).getEdge());
                        }
                        int bestPathEdgeCount = mergedEdgeIds.size();

                        // Add routed segment edges
                        for (EdgeIteratorState e : allRoutedEdges) {
                            mergedEdgeIds.add(resolveToRealEdge(e).getEdge());
                        }
                        int routedEdgeCount = mergedEdgeIds.size() - bestPathEdgeCount;

                        System.out.println("Via-waypoint routing: merged edges = " + mergedEdgeIds.size() +
                                " (best path: " + bestPathEdgeCount + " + routed segments: " + routedEdgeCount + ")" +
                                ", total routed distance=" + String.format("%.1f", totalRoutedDistance) + "m");

                        // Check if all observations now snap to the merged edge set
                        List<List<Snap>> viaSnapsPerObservation = new ArrayList<>();
                        boolean allOnMergedPath = true;
                        int missedCount = 0;
                        for (int obsIdx = 0; obsIdx < snapsPerObservationTmp.size(); obsIdx++) {
                            List<Snap> snaps = snapsPerObservationTmp.get(obsIdx);
                            boolean found = false;
                            for (Snap snap : snaps) {
                                if (mergedEdgeIds.contains(snap.getClosestEdge().getEdge())) {
                                    viaSnapsPerObservation.add(Collections.singletonList(snap));
                                    found = true;
                                    break;
                                }
                            }
                            if (!found) {
                                missedCount++;
                                Observation missedObs = filteredObservations.get(obsIdx);
                                System.out.println("  Observation " + missedObs.getPoint().index + " NOT on merged path at " +
                                        missedObs.getPoint().lat + "," + missedObs.getPoint().lon +
                                        " (snaps on edges: " + snaps.stream().map(s ->
                                        s.getClosestEdge().getEdge() + " at " + s.getSnappedPoint().lat + "," + s.getSnappedPoint().lon)
                                        .collect(Collectors.joining("; ")) + ")");
                                allOnMergedPath = false;
                            }
                        }

                        if (allOnMergedPath) {
                            System.out.println("Via-waypoint routing: ALL " + snapsPerObservationTmp.size() +
                                    " observations on merged path, using it for direct routing");
                            snapsPerObservationOnRoutedPath.clear();
                            snapsPerObservationOnRoutedPath.addAll(viaSnapsPerObservation);
                            usedDirectRouting = true;
                            anySnapNotOnAnyRoutedPath = false;
                            routedPath = bestPath;
                            
                            // Print bestPath as GeoJSON for via-waypoint routing
                            if (bestPath != null && bestPath.isFound()) {
                                PointList bestPathPoints = bestPath.calcPoints();
                                StringBuilder geoJson = new StringBuilder();
                                geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
                                for (int i = 0; i < bestPathPoints.size(); i++) {
                                    if (i > 0) geoJson.append(",");
                                    geoJson.append("[").append(bestPathPoints.getLon(i)).append(",").append(bestPathPoints.getLat(i)).append("]");
                                }
                                geoJson.append("]},\"properties\":{\"stroke\":\"#00ff00\",\"path_index\":")
                                        .append(bestPathIndex)
                                        .append(",\"distance\":")
                                        .append(bestPath.getDistance())
                                        .append(",\"snaps\":")
                                        .append(maxSnapsCount)
                                        .append(",\"selected\":true,\"via_waypoint\":true}}");
                                System.out.println("BestPath GeoJSON (via-waypoint): " + geoJson);
                            }
                            
                            statistics.put("usedDirectRouting", true);
                            statistics.put("usedViaWaypointRouting", true);
                        } else {
                            System.out.println("Via-waypoint routing: " + missedCount + " observations not on merged path, " +
                                    "falling back to default matching");
                            statistics.put("usedViaWaypointRouting", false);
                        }
                    } else {
                        System.out.println("Via-waypoint routing: could not route all segments, falling back to default matching");
                        statistics.put("usedViaWaypointRouting", false);
                    }
                } else {
                    System.out.println("Via-waypoint routing: some waypoints have no snaps, falling back to default matching");
                    statistics.put("usedViaWaypointRouting", false);
                }
            }
        }
        if (anySnapNotOnAnyRoutedPath || routedPath == null) {
            // Creates candidates from the Snaps of all observations (a candidate is basically a
            // Snap + direction). We need to put lower the accuracy to a max value of 300
            List<List<Snap>> snapsPerObservation = filteredObservations.stream()
                    .map(o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, Math.min(o.getPoint().accuracy, 300.0),o.getPoint().index, o.getPoint().timestamp))
                    .collect(Collectors.toList());
            statistics.put("snapsPerObservation", snapsPerObservation.stream().mapToInt(Collection::size).toArray());

            // Create the query graph, containing split edges so that all the places where an observation might have happened
            // are a node. This modifies the Snap objects and puts the new node numbers into them.
            queryGraph = QueryGraph.create(graph, snapsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
            List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, snapsPerObservation);
            seq = computeViterbiSequence(timeSteps, ignoreErrors, sw);
            statistics.put("snapDistanceRanks", IntStream.range(0, seq.size()).map(i -> snapsPerObservation.get(i).indexOf(seq.get(i).state.getSnap())).toArray());
            statistics.put("maxSnapDistances", IntStream.range(0, seq.size()).mapToDouble(i -> snapsPerObservation.get(i).stream().mapToDouble(Snap::getQueryDistance).max().orElse(-1.0)).toArray());

        } else {
            // remove from filteredObsevations, those which go back on the path
            filteredObservations = filteredObservations.stream().filter(o ->
                    snapsPerObservationOnRoutedPath.stream().anyMatch(s ->
                            s.get(0).getQueryPoint().equals(new GHPoint(o.getPoint().lat, o.getPoint().lon))
                    )
            ).collect(Collectors.toList());
            statistics.put("snapsPerObservation", snapsPerObservationOnRoutedPath.stream().mapToInt(Collection::size).toArray());

            // Create the query graph, containing split edges so that all the places where an observation might have happened
            // are a node. This modifies the Snap objects and puts the new node numbers into them.
            queryGraph = QueryGraph.create(graph, snapsPerObservationOnRoutedPath.stream().flatMap(Collection::stream).collect(Collectors.toList()));
            List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, snapsPerObservationOnRoutedPath);
            seq = computeViterbiSequence(timeSteps, ignoreErrors, sw);
            statistics.put("snapDistanceRanks", IntStream.range(0, seq.size()).map(i -> snapsPerObservationOnRoutedPath.get(i).indexOf(seq.get(i).state.getSnap())).toArray());
            statistics.put("maxSnapDistances", IntStream.range(0, seq.size()).mapToDouble(i -> snapsPerObservationOnRoutedPath.get(i).stream().mapToDouble(Snap::getQueryDistance).max().orElse(-1.0)).toArray());

        }

        // Compute the most likely sequence of map matching candidates:
        statistics.put("transitionDistances", seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> Math.round(s.transitionDescriptor.getDistance())).toArray());
        statistics.put("visitedNodes", router.getVisitedNodes());
        statistics.put("snapDistances", seq.stream().mapToDouble(s -> s.state.getSnap().getQueryDistance()).toArray());

        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());

        result = new MatchResult(prepareEdgeMatches(seq));
        Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
        result.setMergedPath(new MapMatchedPath(queryGraph, queryGraphWeighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> s.transitionDescriptor.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(queryGraphWeighting);
        return result;
    }
}