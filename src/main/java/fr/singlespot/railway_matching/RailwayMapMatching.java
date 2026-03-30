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

    // State from prepareQueryGraph(), consumed by match_with_routing()
    private boolean queryGraphPrepared = false;
    private List<List<Snap>> preparedSnapsPerObservation;

    public RailwayMapMatching(BaseGraph graph, LocationIndexTree locationIndex, Router router) {
        super(graph, locationIndex, router);
    }

    public static RailwayMapMatching fromGraphHopper(GraphHopper graphHopper, PMap hints) {
        Router router = routerFromGraphHopper(graphHopper, hints);
        return new RailwayMapMatching(graphHopper.getBaseGraph(), (LocationIndexTree) graphHopper.getLocationIndex(), router);
    }

    /**
     * Prepare the unified QueryGraph from all observation snaps.
     * Call this BEFORE routeGap and match_with_routing so that all routing
     * happens on the same graph. The returned QueryGraph should be passed to
     * routeGap for initial routing; match_with_routing will automatically
     * reuse the prepared graph and snaps.
     *
     * @return the unified QueryGraph containing virtual nodes for all observation snaps
     */
    public QueryGraph prepareQueryGraph(List<Observation> observations, int offset) {
        List<Observation> observationSubList = observations.subList(offset, observations.size());
        List<Observation> preparedFilteredObservations = filterObservations(observationSubList);

        java.util.function.Function<Observation, List<Snap>> findCandidateSnaps =
                o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon,
                        Math.max(20, o.getPoint().accuracy), o.getPoint().index, o.getPoint().timestamp);

        preparedSnapsPerObservation = preparedFilteredObservations.stream()
                .map(findCandidateSnaps)
                .collect(Collectors.toList());

        queryGraph = QueryGraph.create(graph,
                preparedSnapsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        queryGraphPrepared = true;

        System.out.println("Prepared unified QueryGraph with " + preparedSnapsPerObservation.size() +
                " observations, " + preparedSnapsPerObservation.stream().mapToInt(List::size).sum() + " total snaps");
        return queryGraph;
    }

    /**
     * Get snaps for the first filtered observation (after prepareQueryGraph).
     * These snaps are on the unified QueryGraph and can be used as routing start points.
     */
    public List<Snap> getFirstObservationSnaps() {
        if (!queryGraphPrepared || preparedSnapsPerObservation.isEmpty()) return Collections.emptyList();
        return preparedSnapsPerObservation.get(0);
    }

    /**
     * Get snaps for the last filtered observation (after prepareQueryGraph).
     * These snaps are on the unified QueryGraph and can be used as routing end points.
     */
    public List<Snap> getLastObservationSnaps() {
        if (!queryGraphPrepared || preparedSnapsPerObservation.isEmpty()) return Collections.emptyList();
        return preparedSnapsPerObservation.get(preparedSnapsPerObservation.size() - 1);
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
     * @param routedPaths  The list of routed paths between the first and last observation.
     *                     These MUST be routed on the same queryGraph returned by
     *                     {@link #prepareQueryGraph(List, int)} to ensure a unified graph context.
     * @param forceInitialRouting Whether to force using the best initial route even if not all
     *                           observations are on it
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
        java.util.function.Function<Observation, List<Snap>> findCandidateSnaps = o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, Math.max(20, o.getPoint().accuracy), o.getPoint().index, o.getPoint().timestamp);

        List<List<Snap>> snapsPerObservationTmp;
        if (queryGraphPrepared) {
            // Reuse snaps and queryGraph from prepareQueryGraph()
            snapsPerObservationTmp = preparedSnapsPerObservation;
            queryGraphPrepared = false; // consumed
        } else {
            snapsPerObservationTmp = filteredObservations.stream()
                    .map(findCandidateSnaps)
                    .collect(Collectors.toList());
            queryGraph = QueryGraph.create(graph, snapsPerObservationTmp.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        }

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
                // Get the set to store edge indices for this path
                Set<Integer> pathEdgeIndices = routedPathsPathEdgeIndices.get(routedPathsIndex);
                // Skip invalid paths
                if (!tmpRoutedPath.isFound()) {
                    continue;
                }
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
                    // Initialize bestPathSnaps with empty lists for all observations
                    bestPathSnaps = new ArrayList<>();
                    List<List<Snap>> pathSnaps = snapsPerObservationOnRoutedPathTmpList.get(routedPathsIndex);
                    for (int i = 0; i < filteredObservations.size(); i++) {
                        bestPathSnaps.add(new ArrayList<>());
                    }
                    // Add snaps from this path to the corresponding filtered observation indices
                    for (List<Snap> pathSnap : pathSnaps) {
                        if (!pathSnap.isEmpty()) {
                            int queryPointIndex = pathSnap.get(0).getQueryPoint().index;
                            // Find the filtered observation index that matches this queryPoint
                            for (int j = 0; j < filteredObservations.size(); j++) {
                                if (filteredObservations.get(j).getPoint().index == queryPointIndex) {
                                    bestPathSnaps.get(j).addAll(pathSnap);
                                    break;
                                }
                            }
                        }
                    }
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
                    usedDirectRouting = true;
                    break;
                }
            }
// Print selected path as GeoJSON for direct routing
            if (bestPath != null && bestPath.isFound()) {
                PointList pathPoints = bestPath.calcPoints();
                StringBuilder geoJson = new StringBuilder();
                geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
                for (int i = 0; i < pathPoints.size(); i++) {
                    if (i > 0) geoJson.append(",");
                    geoJson.append("[").append(pathPoints.getLon(i)).append(",").append(pathPoints.getLat(i)).append("]");
                }
                geoJson.append("]},\"properties\":{\"stroke\":\"#0000ff\",\"path_index\":")
                        .append(bestPathIndex)
                        .append(",\"distance\":")
                        .append(bestPath.getDistance())
                        .append(",\"snaps\":")
                        .append(bestPathSnaps.size())
                        .append(",\"selected\":true,\"direct_routing\":true}}");
                System.out.println("BestPath GeoJSON (direct): " + geoJson);
            }
            // If forcing routing and we found a best path, use it
            if (forceInitialRouting && bestPath != null) {
                routedPath = bestPath;
                snapsPerObservationOnRoutedPath.addAll(bestPathSnaps);
                System.out.println("Forced routing - SELECTED path #" + (bestPathIndex + 1) + " with " + maxSnapsCount +
                        " snaps out of " + snapsPerObservationTmp.size() + " for map matching");

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

            // Print observations not on best path (if best path exists)
            if (bestPath != null && bestPath.isFound()) {
                for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
                    List<Boolean> snapNotOnRoutedPath = snapsNotOnRoutedPaths.get(observationsIndex);
                    List<Snap> snaps = snapsPerObservationTmp.get(observationsIndex);
                    // Check if this observation is not on the best path specifically
                    if (observationsIndex < snapNotOnRoutedPath.size() && snapNotOnRoutedPath.get(bestPathIndex) && !snaps.isEmpty()) {
                        System.out.println("Observation not on best path #" + (bestPathIndex + 1) + ": " + snapsPerObservationTmp.get(observationsIndex).get(0).getQueryPoint());
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
                        " off-path observations (relative to best path #" + (bestPathIndex) + ")" +
                        " (was " + observationsNotOnAnyPathIndices.size() + " off all paths)");

                // Build set for quick lookup
                Set<Integer> offPathSet = new LinkedHashSet<>(observationsNotOnBestPathIndices);

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
                    int firstObsOriginalIdx = filteredObservations.get(seg.get(0)).getPoint().index;
                    int lastObsOriginalIdx = filteredObservations.get(seg.get(seg.size() - 1)).getPoint().index;
                    
                    // Find anchor before and after using original observation indices
                    Integer anchorBeforeOriginalIdx = null;
                    Integer anchorAfterOriginalIdx = null;
                    
                    // Look for the closest on-path observation before this segment
                    for (int i = seg.get(0) - 1; i >= 0; i--) {
                        if (!offPathSet.contains(i)) {
                            anchorBeforeOriginalIdx = filteredObservations.get(i).getPoint().index;
                            break;
                        }
                    }
                    
                    // Look for the closest on-path observation after this segment  
                    for (int i = seg.get(seg.size() - 1) + 1; i < filteredObservations.size(); i++) {
                        if (!offPathSet.contains(i)) {
                            anchorAfterOriginalIdx = filteredObservations.get(i).getPoint().index;
                            break;
                        }
                    }
                    
                    System.out.println("  Segment " + segNum + ": off-path obs " + firstObsOriginalIdx + "-" + lastObsOriginalIdx +
                            " (anchor before: obs " + (anchorBeforeOriginalIdx != null ? anchorBeforeOriginalIdx : "NONE") +
                            ", anchor after: obs " + (anchorAfterOriginalIdx != null ? anchorAfterOriginalIdx : "NONE") + ")");
                }

                // Build waypoint lists for each segment: [anchorBefore, offPath1, ..., offPathN, anchorAfter]
                List<List<Integer>> segmentWaypointIndices = new ArrayList<>();
                boolean allWaypointsHaveSnaps = true;

                for (int segNum = 0; segNum < offPathSegments.size(); segNum++) {
                    List<Integer> seg = offPathSegments.get(segNum);
                    List<Integer> waypoints = new ArrayList<>();

                    // Add anchor before (last on-path obs before segment) - use original observation index
                    Integer anchorBeforeOriginalIdx = null;
                    for (int i = seg.get(0) - 1; i >= 0; i--) {
                        if (!offPathSet.contains(i)) {
                            anchorBeforeOriginalIdx = filteredObservations.get(i).getPoint().index;
                            break;
                        }
                    }
                    if (anchorBeforeOriginalIdx != null) {
                        waypoints.add(anchorBeforeOriginalIdx);
                    } else {
                        // First observation is off-path, use it as its own start
                        System.out.println("  Segment " + segNum + ": no on-path anchor before, first obs is off-path");
                    }

                    // Add all off-path observations in this segment - use original observation indices
                    for (int offPathIdx : seg) {
                        waypoints.add(filteredObservations.get(offPathIdx).getPoint().index);
                    }

                    // Add anchor after (first on-path obs after segment) - use original observation index
                    Integer anchorAfterOriginalIdx = null;
                    for (int i = seg.get(seg.size() - 1) + 1; i < filteredObservations.size(); i++) {
                        if (!offPathSet.contains(i)) {
                            anchorAfterOriginalIdx = filteredObservations.get(i).getPoint().index;
                            break;
                        }
                    }
                    if (anchorAfterOriginalIdx != null) {
                        waypoints.add(anchorAfterOriginalIdx);
                    } else {
                        // Last observation is off-path, use it as its own end
                        System.out.println("  Segment " + segNum + ": no on-path anchor after, last obs is off-path");
                    }

                    segmentWaypointIndices.add(waypoints);
                }

                // Use existing snaps from snapsPerObservationTmp
                Map<Integer, List<Snap>> waypointAllSnapsMap = new LinkedHashMap<>();
                for (List<Integer> waypoints : segmentWaypointIndices) {
                    for (int obsIdx : waypoints) {
                        if (waypointAllSnapsMap.containsKey(obsIdx)) continue;
                        // Find the snap directly by matching the original observation index
                        List<Snap> candidateSnaps = null;
                        Observation obs = null;
                        for (List<Snap> snapList : snapsPerObservationTmp) {
                            if (!snapList.isEmpty() && snapList.get(0).getQueryPoint().index == obsIdx) {
                                candidateSnaps = snapList;
                                // Find the corresponding observation
                                for (Observation filteredObs : filteredObservations) {
                                    if (filteredObs.getPoint().index == obsIdx) {
                                        obs = filteredObs;
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                        if (candidateSnaps == null || obs == null) {
                            System.out.println("  Obs " + obsIdx + ": NOT FOUND IN SNAPS OR OBSERVATIONS - aborting via-waypoint routing");
                            allWaypointsHaveSnaps = false;
                            break;
                        }

                        if (candidateSnaps.isEmpty()) {
                            System.out.println("  Obs " + obs.getPoint().index + ": NO SNAPS at " +
                                    obs.getPoint().lat + "," + obs.getPoint().lon +
                                    " - aborting via-waypoint routing");
                            allWaypointsHaveSnaps = false;
                            break;
                        }
                        waypointAllSnapsMap.put(obsIdx, candidateSnaps);
                        Snap closestSnap = candidateSnaps.get(0);
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
                    // We don't need a new QueryGraph, we can reuse queryGraph which already has the virtual nodes for all snaps

                    // Pre-compute bestPath edges and node set for splice logic
                    List<EdgeIteratorState> bestPathEdges = bestPath.calcEdges();
                    Set<Integer> bestPathNodeSet = new HashSet<>();
                    if (!bestPathEdges.isEmpty()) {
                        int prevNode = bestPathEdges.get(0).getBaseNode();
                        bestPathNodeSet.add(prevNode);
                        for (EdgeIteratorState e : bestPathEdges) {
                            int nextNode = (e.getBaseNode() == prevNode) ? e.getAdjNode() : e.getBaseNode();
                            bestPathNodeSet.add(nextNode);
                            prevNode = nextNode;
                        }
                    }

                    // Route each segment and collect edges
                    List<EdgeIteratorState> allRoutedEdges = new ArrayList<>();
                    // Track edges per segment for building the spliced ordered edge list
                    List<List<EdgeIteratorState>> perSegmentRoutedEdges = new ArrayList<>();
                    // Track actual start/end nodes for each segment (for node-based splicing)
                    List<int[]> segmentBoundaryNodes = new ArrayList<>();
                    boolean allSegmentsRouted = true;
                    List<List<Snap>> routedPathSnaps = new ArrayList<>();
                    // Initialize routedPathSnaps with empty lists for each observation
                    for (int i = 0; i < filteredObservations.size(); i++) {
                        routedPathSnaps.add(new ArrayList<>());
                    }
                    // Map original observation index -> filtered position for correct routedPathSnaps indexing
                    Map<Integer, Integer> originalToFilteredPos = new HashMap<>();
                    for (int i = 0; i < filteredObservations.size(); i++) {
                        originalToFilteredPos.put(filteredObservations.get(i).getPoint().index, i);
                    }

                    for (int segNum = 0; segNum < segmentWaypointIndices.size(); segNum++) {
                        List<Integer> waypoints = segmentWaypointIndices.get(segNum);
                        List<EdgeIteratorState> segmentEdges = new ArrayList<>();
                        int segmentStartNode = -1;
                        int segmentEndNode = -1;
                        StringBuilder wpDesc = new StringBuilder();
                        for (int wi = 0; wi < waypoints.size(); wi++) {
                            if (wi > 0) wpDesc.append(" -> ");
                            wpDesc.append(waypoints.get(wi));
                        }
                        System.out.println("Via-waypoint routing segment " + segNum + ": routing through " +
                                waypoints.size() + " waypoints (obs indices: " + wpDesc + ")");

                        Snap previousLegToSnap = null; // chain consecutive legs
                        boolean segmentSpliceable = true; // track if segment remains spliceable
                        for (int wpIdx = 0; wpIdx < waypoints.size() - 1 && segmentSpliceable; wpIdx++) {
                            int fromObsIdx = waypoints.get(wpIdx);
                            int toObsIdx = waypoints.get(wpIdx + 1);
                            List<Snap> allFromCandidates = waypointAllSnapsMap.get(fromObsIdx);
                            List<Snap> toCandidates = waypointAllSnapsMap.get(toObsIdx);
                            int maxCandidates = 10;
                            if (allFromCandidates.size() > maxCandidates)
                                allFromCandidates = allFromCandidates.subList(0, maxCandidates);
                            if (toCandidates.size() > maxCandidates)
                                toCandidates = toCandidates.subList(0, maxCandidates);

                            // Build from-candidate list: prefer chaining from previous leg's toSnap
                            List<Snap> fromCandidates;
                            if (wpIdx > 0 && previousLegToSnap != null) {
                                // Try chained snap first, then fall back to all candidates
                                fromCandidates = new ArrayList<>();
                                fromCandidates.add(previousLegToSnap);
                                for (Snap s : allFromCandidates) {
                                    if (s.getClosestNode() != previousLegToSnap.getClosestNode()) {
                                        fromCandidates.add(s);
                                    }
                                }
                            } else {
                                fromCandidates = allFromCandidates;
                            }

                            
                            // Calculate the direct distance between the two observations to use as a baseline for a "suitable" path
                            GHPoint fromPoint = fromCandidates.get(0).getQueryPoint();
                            GHPoint toPoint = toCandidates.get(0).getQueryPoint();
                            double directDistance = DistanceCalcEarth.DIST_EARTH.calcDist(
                                    fromPoint.lat, fromPoint.lon, toPoint.lat, toPoint.lon);
                            double suitableDistanceThreshold = Math.max(directDistance * 2.0, directDistance + 2000.0);

                            // Try from-snap × to-snap combinations and pick shortest path
                            Path bestLegPath = null;
                            Snap bestFromSnap = null;
                            Snap bestToSnap = null;
                            boolean suitablePathFound = false;
                            for (Snap fromSnap : fromCandidates) {
                                int fromNode = fromSnap.getClosestNode();
                                for (Snap toSnap : toCandidates) {
                                    int toNode = toSnap.getClosestNode();
                                    if (fromNode == toNode) continue;
                                    try {
                                        List<Path> legPaths = router.calcPaths(queryGraph, fromNode, EdgeIterator.ANY_EDGE,
                                                new int[]{toNode}, new int[]{EdgeIterator.ANY_EDGE});
                                        if (!legPaths.isEmpty() && legPaths.get(0).isFound()) {
                                            Path candidate = legPaths.get(0);
                                            if (bestLegPath == null || candidate.getDistance() < bestLegPath.getDistance()) {
                                                bestLegPath = candidate;
                                                bestFromSnap = fromSnap;
                                                bestToSnap = toSnap;

                                                if (candidate.getDistance() <= suitableDistanceThreshold) {
                                                    suitablePathFound = true;
                                                    break;
                                                }
                                            }
                                        }
                                    } catch (Exception e) {
                                        // Skip failed combinations
                                    }
                                }
                                if (suitablePathFound) break;
                            }

                            if (!suitablePathFound) {
                                System.out.println("    -> FAILED: no suitable path found (tried " +
                                        fromCandidates.size() + " from × " + toCandidates.size() + " to snap combinations)");
                                allSegmentsRouted = false;
                                break;
                            }

                            // Track boundary nodes for the segment
                            if (wpIdx == 0) segmentStartNode = bestFromSnap.getClosestNode();
                            if (wpIdx == waypoints.size() - 2) segmentEndNode = bestToSnap.getClosestNode();

                            // Add best snaps to routedPathSnaps (using filtered position, not original index)
                            Integer fromFilteredPos = originalToFilteredPos.get(fromObsIdx);
                            Integer toFilteredPos = originalToFilteredPos.get(toObsIdx);
                            if (fromFilteredPos != null) routedPathSnaps.get(fromFilteredPos).add(bestFromSnap);
                            if (toFilteredPos != null) routedPathSnaps.get(toFilteredPos).add(bestToSnap);

                            // If chaining fell through to a different snap, bridge the gap
                            if (wpIdx > 0 && previousLegToSnap != null
                                    && bestFromSnap.getClosestNode() != previousLegToSnap.getClosestNode()) {
                                int prevEndNode = previousLegToSnap.getClosestNode();
                                int curStartNode = bestFromSnap.getClosestNode();
                                double intraBridgeMax = 20000; // 20km max for intra-segment bridge
                                try {
                                    List<Path> intraBridge = router.calcPaths(queryGraph, prevEndNode, EdgeIterator.ANY_EDGE,
                                            new int[]{curStartNode}, new int[]{EdgeIterator.ANY_EDGE});
                                    if (!intraBridge.isEmpty() && intraBridge.get(0).isFound()
                                            && intraBridge.get(0).getDistance() <= intraBridgeMax) {
                                        List<EdgeIteratorState> bridgeEdges = intraBridge.get(0).calcEdges();
                                        allRoutedEdges.addAll(bridgeEdges);
                                        segmentEdges.addAll(bridgeEdges);
                                    } else {
                                        // Bridge too long - mark segment as unspliceable
                                        segmentSpliceable = false;
                                        segmentStartNode = -1;
                                        segmentEndNode = -1;
                                    }
                                } catch (Exception e) {
                                    // Bridge failed - mark segment as unspliceable
                                    segmentSpliceable = false;
                                    segmentStartNode = -1;
                                    segmentEndNode = -1;
                                }
                                if (!segmentSpliceable) break; // exit leg loop if segment is unspliceable
                            }

                            if (!segmentSpliceable) break; // exit leg loop if segment is unspliceable

                            List<EdgeIteratorState> legEdges = bestLegPath.calcEdges();
                            allRoutedEdges.addAll(legEdges);
                            segmentEdges.addAll(legEdges);
                            previousLegToSnap = bestToSnap; // chain next leg from this snap
                        }
                        // Skip splice attempts if segment is already unspliceable
                        if (segmentSpliceable) {
                            // Walk-back splice: if the segment boundary node isn't on the bestPath,
                            // walk back (start) or forward (end) to find an on-path observation whose
                            // bestPathSnap node IS on the bestPath, then route a bridge to connect.
                            List<Integer> segWaypoints = segmentWaypointIndices.get(segNum);

                            // --- Start anchor walk-back ---
                            // If the segment's start node isn't on the bestPath, walk back from the
                            // anchor (inclusive) to find an on-path observation that can bridge to it.
                            double maxBridgeDistance = 20000; // 20km max bridge
                            if (segmentStartNode >= 0 && !bestPathNodeSet.contains(segmentStartNode)) {
                                int anchorOrigIdx = segWaypoints.get(0);
                                Integer anchorFiltPos = originalToFilteredPos.get(anchorOrigIdx);
                                boolean spliceFound = false;
                                if (anchorFiltPos != null) {
                                    for (int wb = anchorFiltPos; wb >= 0 && !spliceFound; wb--) {
                                        if (offPathSet.contains(wb)) continue;
                                        List<Snap> bpSnaps = bestPathSnaps.get(wb);
                                        if (bpSnaps.isEmpty()) continue;
                                        for (Snap bpSnap : bpSnaps) {
                                            int spliceNode = bpSnap.getClosestNode();
                                            if (!bestPathNodeSet.contains(spliceNode)) continue;
                                            try {
                                                List<Path> bridge = router.calcPaths(queryGraph, spliceNode, EdgeIterator.ANY_EDGE,
                                                        new int[]{segmentStartNode}, new int[]{EdgeIterator.ANY_EDGE});
                                                if (!bridge.isEmpty() && bridge.get(0).isFound()
                                                        && bridge.get(0).getDistance() <= maxBridgeDistance) {
                                                    List<EdgeIteratorState> bridgeEdges = bridge.get(0).calcEdges();
                                                    segmentEdges.addAll(0, bridgeEdges);
                                                    System.out.println("  Walk-back splice (start): obs " +
                                                            filteredObservations.get(wb).getPoint().index +
                                                            " -> segment start, bridge=" + bridgeEdges.size() +
                                                            " edges, " + String.format("%.0f", bridge.get(0).getDistance()) + "m");
                                                    segmentStartNode = spliceNode;
                                                    spliceFound = true;
                                                    break;
                                                }
                                            } catch (Exception e) { /* skip */ }
                                        }
                                    }
                                }
                                if (!spliceFound) {
                                    System.out.println("  WARNING: could not find walk-back splice for start of segment " + segNum +
                                            " — segment will be excluded from merge");
                                    segmentStartNode = -1; // mark unspliceable
                                }
                            }

                            // --- End anchor walk-forward ---
                            if (segmentEndNode >= 0 && !bestPathNodeSet.contains(segmentEndNode)) {
                                int anchorOrigIdx = segWaypoints.get(segWaypoints.size() - 1);
                                Integer anchorFiltPos = originalToFilteredPos.get(anchorOrigIdx);
                                boolean spliceFound = false;
                                if (anchorFiltPos != null) {
                                    for (int wf = anchorFiltPos; wf < filteredObservations.size() && !spliceFound; wf++) {
                                    if (offPathSet.contains(wf)) continue;
                                    List<Snap> bpSnaps = bestPathSnaps.get(wf);
                                    if (bpSnaps.isEmpty()) continue;
                                    for (Snap bpSnap : bpSnaps) {
                                        int spliceNode = bpSnap.getClosestNode();
                                        if (!bestPathNodeSet.contains(spliceNode)) continue;
                                        try {
                                            List<Path> bridge = router.calcPaths(queryGraph, segmentEndNode, EdgeIterator.ANY_EDGE,
                                                    new int[]{spliceNode}, new int[]{EdgeIterator.ANY_EDGE});
                                            if (!bridge.isEmpty() && bridge.get(0).isFound()
                                                    && bridge.get(0).getDistance() <= maxBridgeDistance) {
                                                List<EdgeIteratorState> bridgeEdges = bridge.get(0).calcEdges();
                                                segmentEdges.addAll(bridgeEdges);
                                                System.out.println("  Walk-forward splice (end): segment end -> obs " +
                                                        filteredObservations.get(wf).getPoint().index +
                                                        ", bridge=" + bridgeEdges.size() +
                                                        " edges, " + String.format("%.0f", bridge.get(0).getDistance()) + "m");
                                                segmentEndNode = spliceNode;
                                                spliceFound = true;
                                                break;
                                            }
                                        } catch (Exception e) { /* skip */ }
                                    }
                                }
                                }
                                if (!spliceFound) {
                                    System.out.println("  WARNING: could not find walk-forward splice for end of segment " + segNum +
                                            " — segment will be excluded from merge");
                                    segmentEndNode = -1; // mark unspliceable
                                }
                            }
                        } // end of splice attempts

                        perSegmentRoutedEdges.add(segmentEdges);
                        segmentBoundaryNodes.add(new int[]{segmentStartNode, segmentEndNode});
                        if (!allSegmentsRouted) break;
                    }

                    if (allSegmentsRouted && !allRoutedEdges.isEmpty()) {
                        // Via-waypoint routing succeeded — all paths are on the unified queryGraph.
                        // Build merged path by splicing bestPath edges with routed detour segments.
                        List<EdgeIteratorState> mergedPath = buildMergedEdgeList(
                                bestPathEdges, perSegmentRoutedEdges, segmentBoundaryNodes);

                        System.out.println("Via-waypoint routing: successfully routed detour segments. Total merged edges=" + mergedPath.size());

                        // Build edge matches from the full merged path (including detour edges)
                        List<EdgeMatch> edgeMatches = buildEdgeMatchesForMergedPath(
                                mergedPath, observations, filteredObservations,
                                bestPathSnaps, routedPathSnaps);

                        // Via-waypoint routing produced a valid result — return it
                        // directly instead of falling through to Viterbi
                        statistics.put("usedDirectRouting", false);
                        statistics.put("forcedDirectRouting", false);
                        statistics.put("usedViaWaypointRouting", true);
                        statistics.put("visitedNodes", router.getVisitedNodes());

                        // Mark all observations as processed so the caller adds the result
                        processedUpTo = observations.size() - 1;

                        // Print final path as GeoJSON for debugging
                        printMergedPathGeoJson(mergedPath, observations.size(),
                                edgeMatches.stream().mapToInt(em -> em.getStates().size()).sum());

                        // Build a MapMatchedPath from the merged edges on the unified queryGraph
                        Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
                        Path mergedMapMatchedPath = new MapMatchedPath(queryGraph, queryGraphWeighting, mergedPath);

                        result = new MatchResult(edgeMatches);
                        result.setMergedPath(mergedMapMatchedPath);
                        result.setMatchMillis(mergedMapMatchedPath.getTime());
                        result.setMatchLength(mergedMapMatchedPath.getDistance());
                        result.setGPXEntriesLength(gpxLength(observations));
                        result.setGraph(queryGraph);
                        result.setWeighting(queryGraphWeighting);
                        return result;
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
                    .map(o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, Math.min(o.getPoint().accuracy, 300.0), o.getPoint().index, o.getPoint().timestamp))
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
            int beforeFilterCount = filteredObservations.size();

            // To use inside lambdas, we need an effectively final reference
            final List<List<Snap>> finalSnapsOnRoutedPath = snapsPerObservationOnRoutedPath;

            List<Observation> droppedObservations = filteredObservations.stream().filter(o ->
                    finalSnapsOnRoutedPath.stream().noneMatch(s ->
                            !s.isEmpty() && s.get(0).getQueryPoint().equals(new GHPoint(o.getPoint().lat, o.getPoint().lon))
                    )
            ).collect(Collectors.toList());
            if (!droppedObservations.isEmpty()) {
                System.out.println("[DIAG] Direct routing filter: dropping " + droppedObservations.size() + " observations out of " + beforeFilterCount);
                for (Observation dropped : droppedObservations) {
                    System.out.println("[DIAG]   Dropped obs index=" + dropped.getPoint().index +
                            " at " + dropped.getPoint().lat + "," + dropped.getPoint().lon);
                }
            }
            filteredObservations = filteredObservations.stream().filter(o ->
                    finalSnapsOnRoutedPath.stream().anyMatch(s ->
                            !s.isEmpty() && s.get(0).getQueryPoint().equals(new GHPoint(o.getPoint().lat, o.getPoint().lon))
                    )
            ).collect(Collectors.toList());

            // Filter out empty snaps lists too so the sizes match
            snapsPerObservationOnRoutedPath.removeIf(List::isEmpty);

            System.out.println("[DIAG] Direct routing: filteredObservations=" + filteredObservations.size() +
                    ", snapsPerObservationOnRoutedPath=" + snapsPerObservationOnRoutedPath.size());
            if (filteredObservations.size() != snapsPerObservationOnRoutedPath.size()) {
                System.out.println("[DIAG] WARNING: SIZE MISMATCH! filteredObservations=" + filteredObservations.size() +
                        " vs snapsPerObservationOnRoutedPath=" + snapsPerObservationOnRoutedPath.size());
            }
            // Log each observation and its snap details before Viterbi
            for (int diagIdx = 0; diagIdx < Math.min(filteredObservations.size(), snapsPerObservationOnRoutedPath.size()); diagIdx++) {
                Observation diagObs = filteredObservations.get(diagIdx);
                List<Snap> diagSnaps = snapsPerObservationOnRoutedPath.get(diagIdx);
                Snap diagSnap = diagSnaps.get(0);
                System.out.println("[DIAG] TimeStep " + diagIdx + ": obs.index=" + diagObs.getPoint().index +
                        " GPS=" + diagObs.getPoint().lat + "," + diagObs.getPoint().lon +
                        " -> snap.queryPoint=" + diagSnap.getQueryPoint().lat + "," + diagSnap.getQueryPoint().lon +
                        " snap.node=" + diagSnap.getClosestNode() +
                        " snap.edge=" + diagSnap.getClosestEdge().getEdge() +
                        " (" + diagSnap.getClosestEdge().getName() + ")" +
                        " snap.dist=" + String.format("%.1f", diagSnap.getQueryDistance()) + "m" +
                        " candidates=" + diagSnaps.size() +
                        " match=" + diagSnap.getQueryPoint().equals(new GHPoint(diagObs.getPoint().lat, diagObs.getPoint().lon)));
            }
            statistics.put("snapsPerObservation", snapsPerObservationOnRoutedPath.stream().mapToInt(Collection::size).toArray());

            // Reuse the ORIGINAL QueryGraph (built at line 74 from all candidate snaps).
            // The snapsPerObservationOnRoutedPath already contains filtered candidates
            // that are on the merged edge set. Their closestNode values were set when
            // the original QueryGraph was created, so they reference the correct virtual
            // nodes in that graph. No need to rebuild the QueryGraph.
            // Run Viterbi on the original QueryGraph with filtered candidates.
            System.out.println("[DIAG] Using original QueryGraph for Viterbi with " +
                    snapsPerObservationOnRoutedPath.size() + " filtered observation snap sets");
            List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, snapsPerObservationOnRoutedPath);
            seq = computeViterbiSequence(timeSteps, ignoreErrors, sw);
            statistics.put("snapDistanceRanks", IntStream.range(0, seq.size()).map(i -> finalSnapsOnRoutedPath.get(i).indexOf(seq.get(i).state.getSnap())).toArray());
            statistics.put("maxSnapDistances", IntStream.range(0, seq.size()).mapToDouble(i -> finalSnapsOnRoutedPath.get(i).stream().mapToDouble(Snap::getQueryDistance).max().orElse(-1.0)).toArray());

        }

        // Compute the most likely sequence of map matching candidates:
        statistics.put("transitionDistances", seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> Math.round(s.transitionDescriptor.getDistance())).toArray());
        statistics.put("visitedNodes", router.getVisitedNodes());
        statistics.put("snapDistances", seq.stream().mapToDouble(s -> s.state.getSnap().getQueryDistance()).toArray());

        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());

        // Print final path as GeoJSON for debugging
        if (!path.isEmpty()) {
            StringBuilder geoJson = new StringBuilder();
            geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
            boolean first = true;
            for (EdgeIteratorState edge : path) {
                PointList edgePoints = edge.fetchWayGeometry(FetchMode.ALL);
                for (int i = 0; i < edgePoints.size(); i++) {
                    if (!first) geoJson.append(",");
                    geoJson.append("[").append(edgePoints.getLon(i)).append(",").append(edgePoints.getLat(i)).append("]");
                    first = false;
                }
            }
            geoJson.append("]},\"properties\":{\"stroke\":\"#ff0000\",\"path_type\":\"final\",\"edges\":")
                    .append(path.size())
                    .append(",\"distance\":")
                    .append(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum())
                    .append("}}");
            System.out.println("Final Path GeoJSON: " + geoJson);
        }

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

    /**
     * Build the merged edge list by splicing bestPath edges with routed detour segment edges.
     * Uses node-based splicing: for each segment, finds where the segment's actual start/end
     * nodes appear in the bestPath's node sequence, then splices at those positions.
     * All edges are from the unified {@link #queryGraph}.
     *
     * @param bestPathEdges          Edges from the best initial route
     * @param perSegmentRoutedEdges  Edges for each routed detour segment
     * @param segmentBoundaryNodes   [startNode, endNode] for each segment (actual nodes used in routing)
     */
    private List<EdgeIteratorState> buildMergedEdgeList(
            List<EdgeIteratorState> bestPathEdges,
            List<List<EdgeIteratorState>> perSegmentRoutedEdges,
            List<int[]> segmentBoundaryNodes) {

        if (bestPathEdges.isEmpty()) {
            List<EdgeIteratorState> all = new ArrayList<>();
            for (List<EdgeIteratorState> seg : perSegmentRoutedEdges) all.addAll(seg);
            return all;
        }

        // Compute bestPath node sequence: node[i] connects to node[i+1] via edge[i]
        int[] bestPathNodes = new int[bestPathEdges.size() + 1];
        bestPathNodes[0] = bestPathEdges.get(0).getBaseNode();
        for (int i = 0; i < bestPathEdges.size(); i++) {
            EdgeIteratorState e = bestPathEdges.get(i);
            // Follow whichever endpoint is NOT the current node
            bestPathNodes[i + 1] = (e.getBaseNode() == bestPathNodes[i]) ? e.getAdjNode() : e.getBaseNode();
        }

        // Build merged path by replacing bestPath sections with routed segments
        List<EdgeIteratorState> mergedPath = new ArrayList<>();
        int bpCursor = 0; // edge index cursor

        for (int segNum = 0; segNum < perSegmentRoutedEdges.size(); segNum++) {
            int startNode = segmentBoundaryNodes.get(segNum)[0];
            int endNode = segmentBoundaryNodes.get(segNum)[1];

            // Skip unspliceable segments — bestPath continues as-is
            if (startNode == -1 || endNode == -1) {
                continue;
            }

            if (startNode >= 0) {
                boolean foundStart = false;
                // Check if cursor is already at the startNode (consecutive segments share boundary)
                if (bpCursor < bestPathEdges.size() && bestPathNodes[bpCursor] == startNode) {
                    foundStart = true; // Already positioned — no bestPath edges to add
                } else {
                    // Include bestPath edges until we reach the segment's start node
                    while (bpCursor < bestPathEdges.size()) {
                        // Check if the NEXT node in the sequence is the start node
                        if (bestPathNodes[bpCursor + 1] == startNode) {
                            // Include this edge (it arrives at startNode)
                            mergedPath.add(bestPathEdges.get(bpCursor));
                            bpCursor++;
                            foundStart = true;
                            break;
                        }
                        mergedPath.add(bestPathEdges.get(bpCursor));
                        bpCursor++;
                    }
                }
                if (!foundStart) {
                    System.out.println("  WARNING: segment " + segNum + " startNode " + startNode +
                            " not found in bestPath from cursor " + bpCursor);
                }
            }

            // Insert routed segment edges
            mergedPath.addAll(perSegmentRoutedEdges.get(segNum));

            if (endNode >= 0) {
                // Skip bestPath edges until we find one that departs from the segment's end node
                while (bpCursor < bestPathEdges.size()) {
                    if (bestPathNodes[bpCursor] == endNode) {
                        break; // Resume from this edge (it departs from endNode)
                    }
                    bpCursor++;
                }
            }
        }

        // Add remaining bestPath edges after the last segment
        while (bpCursor < bestPathEdges.size()) {
            mergedPath.add(bestPathEdges.get(bpCursor));
            bpCursor++;
        }

        // Validate connectivity
        if (mergedPath.size() > 1) {
            int[] mergedNodes = new int[mergedPath.size() + 1];
            mergedNodes[0] = mergedPath.get(0).getBaseNode();
            for (int i = 0; i < mergedPath.size(); i++) {
                EdgeIteratorState e = mergedPath.get(i);
                mergedNodes[i + 1] = (e.getBaseNode() == mergedNodes[i]) ? e.getAdjNode() : e.getBaseNode();
            }
            for (int i = 0; i < mergedPath.size(); i++) {
                EdgeIteratorState e = mergedPath.get(i);
                if (e.getBaseNode() != mergedNodes[i] && e.getAdjNode() != mergedNodes[i]) {
                    System.out.println("WARNING: Merged path discontinuity at edge " + i +
                            ": expected node " + mergedNodes[i] + " but edge " + e.getEdge() +
                            " connects " + e.getBaseNode() + "<->" + e.getAdjNode());
                }
            }
        }

        return mergedPath;
    }

    /**
     * Build EdgeMatch list from the full merged path, associating observations with their
     * closest edges via bestPathSnaps and routedPathSnaps. Consecutive edges that resolve
     * to the same real edge are collapsed into a single EdgeMatch (matching PathMerger's
     * edge_key detail generation).
     */
    private List<EdgeMatch> buildEdgeMatchesForMergedPath(
            List<EdgeIteratorState> mergedPathEdges,
            List<Observation> observations,
            List<Observation> filteredObservations,
            List<List<Snap>> bestPathSnaps,
            List<List<Snap>> routedPathSnaps) {

        // Build set of real edge IDs in the merged path
        Set<Integer> mergedRealEdgeIds = new LinkedHashSet<>();
        for (EdgeIteratorState edge : mergedPathEdges) {
            mergedRealEdgeIds.add(resolveToRealEdge(edge).getEdge());
        }

        // Map: real edge ID -> list of (observation, snap) pairs
        Map<Integer, List<Object[]>> edgeToObsSnap = new LinkedHashMap<>();

        for (Observation obs : observations) {
            int obsIdx = obs.getPoint().index;

            // Find the filtered observation index
            int filteredObsIdx = findFilteredObsIndex(filteredObservations, obsIdx);
            if (filteredObsIdx < 0) continue;

            // Collect candidate snaps from both bestPath and routed segments
            List<Snap> allCandidateSnaps = new ArrayList<>();
            if (filteredObsIdx < bestPathSnaps.size() && !bestPathSnaps.get(filteredObsIdx).isEmpty()) {
                allCandidateSnaps.addAll(bestPathSnaps.get(filteredObsIdx));
            }
            if (filteredObsIdx < routedPathSnaps.size() && !routedPathSnaps.get(filteredObsIdx).isEmpty()) {
                allCandidateSnaps.addAll(routedPathSnaps.get(filteredObsIdx));
            }

            // Find a snap whose edge is in the merged path
            for (Snap snap : allCandidateSnaps) {
                int realEdgeId = resolveToRealEdge(snap.getClosestEdge()).getEdge();
                if (mergedRealEdgeIds.contains(realEdgeId)) {
                    edgeToObsSnap.computeIfAbsent(realEdgeId, k -> new ArrayList<>())
                            .add(new Object[]{obs, snap});
                    break;
                }
            }
        }

        // Build edge matches, collapsing consecutive same-real-edge entries
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        EdgeIteratorState currentRealEdge = null;
        List<State> currentStates = new ArrayList<>();

        for (EdgeIteratorState edge : mergedPathEdges) {
            EdgeIteratorState realEdge = resolveToRealEdge(edge);

            if (currentRealEdge != null && !equalEdges(currentRealEdge, realEdge)) {
                edgeMatches.add(new EdgeMatch(currentRealEdge, currentStates));
                currentStates = new ArrayList<>();
            }
            currentRealEdge = realEdge;

            // Add states for observations on this edge (only on the first virtual edge of a real edge)
            int realEdgeId = realEdge.getEdge();
            List<Object[]> obsSnapPairs = edgeToObsSnap.remove(realEdgeId);
            if (obsSnapPairs != null) {
                for (Object[] pair : obsSnapPairs) {
                    currentStates.add(new State((Observation) pair[0], (Snap) pair[1]));
                }
            }
        }
        if (currentRealEdge != null) {
            edgeMatches.add(new EdgeMatch(currentRealEdge, currentStates));
        }

        return edgeMatches;
    }

    /**
     * Find the index into filteredObservations that matches the given original observation index.
     * Returns -1 if not found.
     */
    private static int findFilteredObsIndex(List<Observation> filteredObservations, int originalObsIdx) {
        for (int j = 0; j < filteredObservations.size(); j++) {
            if (filteredObservations.get(j).getPoint().index == originalObsIdx) {
                return j;
            }
        }
        return -1;
    }

    /**
     * Print the merged path as GeoJSON for debugging.
     */
    private void printMergedPathGeoJson(List<EdgeIteratorState> mergedPath, int observationCount,
                                        int edgeMatchesWithStates) {
        StringBuilder geoJson = new StringBuilder();
        geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        boolean first = true;
        for (EdgeIteratorState edge : mergedPath) {
            PointList edgePoints = edge.fetchWayGeometry(FetchMode.ALL);
            for (int i = 0; i < edgePoints.size(); i++) {
                if (!first) geoJson.append(",");
                geoJson.append("[").append(edgePoints.getLon(i)).append(",").append(edgePoints.getLat(i)).append("]");
                first = false;
            }
        }
        double totalDistance = 0;
        for (EdgeIteratorState edge : mergedPath) totalDistance += edge.getDistance();
        geoJson.append("]},\"properties\":{\"stroke\":\"#ff0000\",\"path_type\":\"via_waypoint_bypass\"")
                .append(",\"edges\":").append(mergedPath.size())
                .append(",\"distance\":").append(totalDistance)
                .append(",\"observations\":").append(observationCount)
                .append(",\"edge_matches_with_states\":").append(edgeMatchesWithStates)
                .append("}}");
        System.out.println("Via-waypoint routing (bypassing Viterbi) Path GeoJSON: " + geoJson);
    }
}