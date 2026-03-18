package fr.singlespot.railway_matching;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.*;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
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
        Set<Integer> mergedPathEdgeKeys = new HashSet<>();
        resetCounters(observations.size(), offset);
        List<Observation> observationSubList = observations.subList(offset, observations.size());
        List<Observation> filteredObservations = filterObservations(observationSubList);
        statistics.put("filteredObservations", filteredObservations.size());

        // Snap observations to links. Generates multiple candidate snaps per observation.
        java.util.function.Function<Observation, List<Snap>> findCandidateSnaps = o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, Math.max(20, o.getPoint().accuracy), o.getPoint().index, o.getPoint().timestamp);
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
                    usedDirectRouting = true;
                    break;
                }
            }
// Print selected path as GeoJSON for direct routing
            if (bestPath.isFound()) {
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
//                System.out.println("BestPath GeoJSON (direct): " + geoJson);
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

                // Snap all waypoint observations - store ALL candidates per observation
                Map<Integer, List<Snap>> waypointAllSnapsMap = new LinkedHashMap<>();
                for (List<Integer> waypoints : segmentWaypointIndices) {
                    for (int obsIdx : waypoints) {
                        if (waypointAllSnapsMap.containsKey(obsIdx)) continue;
                        Observation obs = filteredObservations.get(obsIdx);
                        List<Snap> candidateSnaps = findCandidateSnaps(obs.getPoint().lat, obs.getPoint().lon,
                                Math.min(Math.max(20,obs.getPoint().accuracy), 300.0), obs.getPoint().index, obs.getPoint().timestamp);
                        if (candidateSnaps.isEmpty()) {
                            System.out.println("  Obs " + obs.getPoint().index + ": NO SNAPS at " +
                                    obs.getPoint().lat + "," + obs.getPoint().lon +
                                    " - aborting via-waypoint routing");
                            allWaypointsHaveSnaps = false;
                            break;
                        }
                        waypointAllSnapsMap.put(obsIdx, candidateSnaps);
                        allSegmentSnaps.addAll(candidateSnaps);
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
                    // Build a query graph for all waypoint snaps
                    QueryGraph waypointQueryGraph = QueryGraph.create(graph, allSegmentSnaps);

                    // Route each segment and collect edges
                    List<EdgeIteratorState> allRoutedEdges = new ArrayList<>();
                    // Track edges per segment for building the spliced ordered edge list
                    List<List<EdgeIteratorState>> perSegmentRoutedEdges = new ArrayList<>();
                    boolean allSegmentsRouted = true;
                    double totalRoutedDistance = 0;

                    for (int segNum = 0; segNum < segmentWaypointIndices.size(); segNum++) {
                        List<Integer> waypoints = segmentWaypointIndices.get(segNum);
                        List<EdgeIteratorState> segmentEdges = new ArrayList<>();
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
                            // Limit to top 10 closest snap candidates to avoid combinatorial explosion
                            List<Snap> fromCandidates = waypointAllSnapsMap.get(fromObsIdx);
                            List<Snap> toCandidates = waypointAllSnapsMap.get(toObsIdx);
                            int maxCandidates = 10;
                            if (fromCandidates.size() > maxCandidates) fromCandidates = fromCandidates.subList(0, maxCandidates);
                            if (toCandidates.size() > maxCandidates) toCandidates = toCandidates.subList(0, maxCandidates);

                            System.out.println("  Leg " + wpIdx + ": obs " + fromCandidates.get(0).getQueryPoint().index +
                                    " (" + fromCandidates.size() + " snap candidates)" +
                                    " -> obs " + toCandidates.get(0).getQueryPoint().index +
                                    " (" + toCandidates.size() + " snap candidates)");

                            // Calculate the direct distance between the two observations to use as a baseline for a "suitable" path
                            GHPoint fromPoint = filteredObservations.get(fromObsIdx).getPoint();
                            GHPoint toPoint = filteredObservations.get(toObsIdx).getPoint();
                            double directDistance = DistanceCalcEarth.DIST_EARTH.calcDist(
                                    fromPoint.lat, fromPoint.lon, toPoint.lat, toPoint.lon);
                            // Define "suitable" as path distance <= direct distance * 2.0 (allowing for some detour)
                            // or if direct distance is very small, add a fixed buffer
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
                                        List<Path> legPaths = router.calcPaths(waypointQueryGraph, fromNode, EdgeIterator.ANY_EDGE,
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

                            if (bestLegPath == null) {
                                System.out.println("    -> FAILED: no path found (tried " +
                                        fromCandidates.size() + " from × " + toCandidates.size() + " to snap combinations)");
                                allSegmentsRouted = false;
                                break;
                            }
                            List<EdgeIteratorState> legEdges = bestLegPath.calcEdges();
                            allRoutedEdges.addAll(legEdges);
                            segmentEdges.addAll(legEdges);
                            totalRoutedDistance += bestLegPath.getDistance();
                            System.out.println("    -> OK: " + legEdges.size() + " edges, distance=" +
                                    String.format("%.1f", bestLegPath.getDistance()) + "m, time=" + bestLegPath.getTime() + "ms" +
                                    " (from edge " + bestFromSnap.getClosestEdge().getEdge() +
                                    " to edge " + bestToSnap.getClosestEdge().getEdge() + ")");
                        }
                        perSegmentRoutedEdges.add(segmentEdges);
                        if (!allSegmentsRouted) break;
                    }

                    if (allSegmentsRouted && !allRoutedEdges.isEmpty()) {
                        // Via-waypoint routing succeeded. Build the result DIRECTLY from
                        // the merged path edges, bypassing Viterbi entirely.
                        // Viterbi routes freely through the entire graph between candidates,
                        // which causes massive loops. Instead, we already have the correct
                        // path from the routing — just package it as a MatchResult.

                        // Build merged path: bestPath edges with routed segments
                        // REPLACING (not supplementing) the bestPath section between
                        // each pair of anchors. The routed segments go through the
                        // off-path observations: anchor→offPath1→...→offPathN→anchor.
                        List<EdgeIteratorState> bestPathEdges = bestPath.calcEdges();

                        // Build a map: real edge ID -> first bestPath edge index
                        Map<Integer, Integer> bestPathEdgeIdToIndex = new LinkedHashMap<>();
                        for (int bpIdx = 0; bpIdx < bestPathEdges.size(); bpIdx++) {
                            int edgeId = resolveToRealEdge(bestPathEdges.get(bpIdx)).getEdge();
                            if (!bestPathEdgeIdToIndex.containsKey(edgeId)) {
                                bestPathEdgeIdToIndex.put(edgeId, bpIdx);
                            }
                        }

                        // Find anchor bestPath indices for each segment
                        List<int[]> segmentAnchorBpIndices = new ArrayList<>();
                        for (int segNum = 0; segNum < offPathSegments.size(); segNum++) {
                            List<Integer> waypoints = segmentWaypointIndices.get(segNum);
                            int anchorBeforeObsIdx = waypoints.get(0);
                            int anchorAfterObsIdx = waypoints.get(waypoints.size() - 1);

                            // Find bestPath index of anchor-before's edge
                            int anchorBeforeBpIdx = -1;
                            if (!offPathSet.contains(anchorBeforeObsIdx)) {
                                for (Snap snap : snapsPerObservationTmp.get(anchorBeforeObsIdx)) {
                                    Integer bpIdx = bestPathEdgeIdToIndex.get(snap.getClosestEdge().getEdge());
                                    if (bpIdx != null) { anchorBeforeBpIdx = bpIdx; break; }
                                }
                            }
                            // Find bestPath index of anchor-after's edge
                            int anchorAfterBpIdx = bestPathEdges.size();
                            if (!offPathSet.contains(anchorAfterObsIdx)) {
                                for (Snap snap : snapsPerObservationTmp.get(anchorAfterObsIdx)) {
                                    Integer bpIdx = bestPathEdgeIdToIndex.get(snap.getClosestEdge().getEdge());
                                    if (bpIdx != null) { anchorAfterBpIdx = bpIdx; break; }
                                }
                            }
                            segmentAnchorBpIndices.add(new int[]{anchorBeforeBpIdx, anchorAfterBpIdx});
                            System.out.println("  Segment " + segNum + " anchors: bestPath[" +
                                    anchorBeforeBpIdx + "] -> bestPath[" + anchorAfterBpIdx + "]" +
                                    ", routed edges: " + perSegmentRoutedEdges.get(segNum).size());
                        }

                        // Build merged path by replacing bestPath sections with routed segments
                        List<EdgeIteratorState> mergedPath = new ArrayList<>();
                        int bpCursor = 0;
                        for (int segNum = 0; segNum < offPathSegments.size(); segNum++) {
                            int[] anchors = segmentAnchorBpIndices.get(segNum);

                            // Add bestPath edges up to (but NOT including) anchor-before
                            // The routed segment starts from the anchor-before node
                            while (bpCursor < anchors[0] && bpCursor < bestPathEdges.size()) {
                                mergedPath.add(bestPathEdges.get(bpCursor));
                                bpCursor++;
                            }

                            // Insert ALL routed segment edges (anchor→offpath→...→anchor)
                            mergedPath.addAll(perSegmentRoutedEdges.get(segNum));

                            // Skip bestPath edges between anchors (replaced by routed segment)
                            bpCursor = Math.max(bpCursor, anchors[1]);
                        }
                        // Add remaining bestPath edges after last segment
                        while (bpCursor < bestPathEdges.size()) {
                            mergedPath.add(bestPathEdges.get(bpCursor));
                            bpCursor++;
                        }

                        System.out.println("Via-waypoint routing: using bestPath directly with " +
                                mergedPath.size() + " edges, total routed distance=" +
                                String.format("%.1f", totalRoutedDistance) + "m");

                        // Build EdgeMatch list directly from merged path edges
                        List<EdgeMatch> edgeMatches = new ArrayList<>();
                        for (EdgeIteratorState edge : mergedPath) {
                            EdgeIteratorState realEdge = resolveToRealEdge(edge);
                            edgeMatches.add(new EdgeMatch(realEdge, new ArrayList<>()));
                        }

                        // Build MapMatchedPath from merged edges
                        Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
                        result = new MatchResult(edgeMatches);
                        result.setMergedPath(new MapMatchedPath(queryGraph, queryGraphWeighting, mergedPath));
                        double matchLength = mergedPath.stream().mapToDouble(EdgeIteratorState::getDistance).sum();
                        long matchMillis = mergedPath.stream().mapToLong(e ->
                                GHUtility.calcMillisWithTurnMillis(queryGraphWeighting, e, false, EdgeIterator.NO_EDGE)).sum();
                        result.setMatchMillis(matchMillis);
                        result.setMatchLength(matchLength);
                        result.setGPXEntriesLength(gpxLength(observations));
                        result.setGraph(queryGraph);
                        result.setWeighting(queryGraphWeighting);

                        // Print final path as GeoJSON
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
                        geoJson.append("]},\"properties\":{\"stroke\":\"#ff0000\",\"path_type\":\"via_waypoint_direct\"")
                                .append(",\"edges\":").append(mergedPath.size())
                                .append(",\"distance\":").append(matchLength).append("}}");
                        System.out.println("Final Path GeoJSON: " + geoJson);

                        statistics.put("usedDirectRouting", true);
                        statistics.put("usedViaWaypointRouting", true);
                        statistics.put("matchLength", matchLength);
                        // Mark all points as processed so the caller adds this result
                        processedUpTo = observations.size() - 1;
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
            List<Observation> droppedObservations = filteredObservations.stream().filter(o ->
                    snapsPerObservationOnRoutedPath.stream().noneMatch(s ->
                            s.get(0).getQueryPoint().equals(new GHPoint(o.getPoint().lat, o.getPoint().lon))
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
                    snapsPerObservationOnRoutedPath.stream().anyMatch(s ->
                            s.get(0).getQueryPoint().equals(new GHPoint(o.getPoint().lat, o.getPoint().lon))
                    )
            ).collect(Collectors.toList());
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
            statistics.put("snapDistanceRanks", IntStream.range(0, seq.size()).map(i -> snapsPerObservationOnRoutedPath.get(i).indexOf(seq.get(i).state.getSnap())).toArray());
            statistics.put("maxSnapDistances", IntStream.range(0, seq.size()).mapToDouble(i -> snapsPerObservationOnRoutedPath.get(i).stream().mapToDouble(Snap::getQueryDistance).max().orElse(-1.0)).toArray());

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
}