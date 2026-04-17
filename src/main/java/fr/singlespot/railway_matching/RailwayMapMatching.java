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
import org.jetbrains.annotations.NotNull;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class RailwayMapMatching extends MapMatching {

    /** Maximum number of snap candidates considered per observation during leg routing. */
    private static final int MAX_SNAP_CANDIDATES = 10;
    /** Maximum route distance allowed for an intra-segment bridge or splice bridge (20 km). */
    private static final double MAX_BRIDGE_DISTANCE = 20_000;
    /** Maximum number of times the start anchor may be stepped back when leg 0 fails. */
    private static final int MAX_ANCHOR_BACK_STEPS = 5;

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
        resetCounters(observations.size(), offset);
        List<Observation> filteredObservations = filterObservations(observations.subList(offset, observations.size()));
        statistics.put("filteredObservations", filteredObservations.size());

        // Snap observations to links. Generates multiple candidate snaps per observation.
        java.util.function.Function<Observation, List<Snap>> findCandidateSnaps = o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, Math.max(20, o.getPoint().accuracy), o.getPoint().index, o.getPoint().timestamp);

        // If queryGraph not prepared yet, create it now with all observation snaps
        if (!queryGraphPrepared) {
            prepareQueryGraph(observations, offset);
        }
        
        // Use the unified queryGraph for all operations
        List<List<Snap>> snapsPerObservationTmp = preparedSnapsPerObservation;
        queryGraphPrepared = false; // consumed

        MatchResult result;
        List<SequenceState<State, Observation, Path>> seq;
        Path routedPath = null;
        boolean anySnapNotOnAnyRoutedPath = false;
        List<List<Snap>> snapsPerObservationOnRoutedPath = new ArrayList<>();
        PathAnalysisResult pathAnalysis = null;

        // Check if there is at least one valid routed path
        if (routedPaths.get(0) != null && routedPaths.stream().anyMatch(Path::isFound)) {
            SnappedPointsAnalyzer analyzer = new SnappedPointsAnalyzer();
            SnappedPointsAnalyzer.AnalysisResult analysisResult = analyzer.analyze(
                    routedPaths,
                    filteredObservations,
                    findCandidateSnaps,
                    this::resolveToRealEdge
            );

            // Analyze routed paths to find best path and check for direct path (case 1)
            pathAnalysis = analyzeRoutedPaths(
                    routedPaths, filteredObservations, snapsPerObservationTmp, 
                    analysisResult, forceInitialRouting);

            // Case 1: All observations on a single routed path
            if (pathAnalysis.hasDirectPath) {
                // Check path distance constraint for case 1
                double totalPathDistance = pathAnalysis.directPath.getDistance();
                double maxAllowedDistance = calculateMaxAllowedDistance(filteredObservations);

                if (totalPathDistance <= maxAllowedDistance) {
                    System.out.println("Case 1: All observations on direct path - using direct routing");
                    routedPath = pathAnalysis.directPath;
                    snapsPerObservationOnRoutedPath = pathAnalysis.directPathSnaps;
                    statistics.put("usedDirectRouting", true);
                    statistics.put("forcedDirectRouting", false);
                } else {
                    System.out.println("Case 1: Path distance " + String.format("%.0f", totalPathDistance) +
                            "m exceeds threshold " + String.format("%.0f", maxAllowedDistance) +
                            "m - falling back to Viterbi");
                }
            }
            // Case 2: Try via-waypoint routing with best path
            else if (pathAnalysis.bestPath != null && !forceInitialRouting) {
                System.out.println("Case 2: Attempting via-waypoint routing with best path");
                
                // Check if any observations are not on any path
                for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
                    List<Boolean> snapNotOnRoutedPath = analysisResult.snapsNotOnRoutedPaths.get(observationsIndex);
                    List<Snap> snaps = snapsPerObservationTmp.get(observationsIndex);
                    if (snapNotOnRoutedPath.stream().allMatch(Boolean::booleanValue) && !snaps.isEmpty()) {
                        anySnapNotOnAnyRoutedPath = true;
                        break;
                    }
                }
                
                if (anySnapNotOnAnyRoutedPath) {
                    MatchResult viaWaypointResult = attemptViaWaypointRouting(
                            pathAnalysis.bestPath, pathAnalysis.bestPathIndex,
                            pathAnalysis.bestPathSnaps, filteredObservations,
                            snapsPerObservationTmp, analysisResult.snapsNotOnRoutedPaths,
                            observations, sw);
                    
                    if (viaWaypointResult != null) {
                        return viaWaypointResult;
                    }
                }
            }
            
            // Handle forced routing
            if (forceInitialRouting && pathAnalysis.bestPath != null) {
                System.out.println("Forced routing - using best path");
                routedPath = pathAnalysis.bestPath;
                snapsPerObservationOnRoutedPath = pathAnalysis.bestPathSnaps;
                statistics.put("usedDirectRouting", false);
                statistics.put("forcedDirectRouting", true);
                
                // Print summary
                System.out.println("Path snap counts summary:");
                for (int i = 0; i < analysisResult.snapsOnPathCounts.length; i++) {
                    if (routedPaths.get(i).isFound()) {
                        String marker = (i == pathAnalysis.bestPathIndex) ? " [SELECTED]" : "";
                        System.out.println("  Path #" +
                                (i + 1) + ": " + analysisResult.snapsOnPathCounts[i] + " snaps out of " +
                                snapsPerObservationTmp.size() + marker);
                    }
                }
            }
        }
        
        // If we have a direct path (Case 1), bypass Viterbi and create result directly
        if (routedPath != null && pathAnalysis.hasDirectPath) {
            System.out.println("Creating direct MatchResult bypassing Viterbi algorithm");
            
            // Build edge matches directly from the routed path
            List<EdgeIteratorState> pathEdges = routedPath.calcEdges();
            List<EdgeMatch> edgeMatches = buildEdgeMatchesForMergedPath(
                    pathEdges, observations, filteredObservations,
                    pathAnalysis.directPathSnaps, Collections.emptyList());
            
            // Create and return result
            statistics.put("usedDirectRouting", true);
            statistics.put("forcedDirectRouting", false);
            statistics.put("usedViaWaypointRouting", false);
            statistics.put("visitedNodes", router.getVisitedNodes());
            statistics.put("snapsPerObservation", pathAnalysis.directPathSnaps.stream().mapToInt(Collection::size).toArray());
            
            processedUpTo = observations.size() - 1;
            
            Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
            Path directMapMatchedPath = new MapMatchedPath(queryGraph, queryGraphWeighting, pathEdges);
            
            result = new MatchResult(edgeMatches);
            result.setMergedPath(directMapMatchedPath);
            result.setMatchMillis(directMapMatchedPath.getTime());
            result.setMatchLength(directMapMatchedPath.getDistance());
            result.setGPXEntriesLength(gpxLength(observations));
            result.setGraph(queryGraph);
            result.setWeighting(queryGraphWeighting);
            
            // Print final path as GeoJSON for debugging
            if (!pathEdges.isEmpty()) {
                StringBuilder geoJson = new StringBuilder();
                geoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
                boolean first = true;
                for (EdgeIteratorState edge : pathEdges) {
                    PointList edgePoints = edge.fetchWayGeometry(FetchMode.ALL);
                    for (int i = 0; i < edgePoints.size(); i++) {
                        if (!first) geoJson.append(",");
                        geoJson.append("[").append(edgePoints.getLon(i)).append(",").append(edgePoints.getLat(i)).append("]");
                        first = false;
                    }
                }
                geoJson.append("]},\"properties\":{\"stroke\":\"#00ff00\",\"path_type\":\"direct_bypass_viterbi\",\"edges\":")
                        .append(pathEdges.size())
                        .append(",\"distance\":")
                        .append(directMapMatchedPath.getDistance())
                        .append("}}");
                System.out.println("Direct Path (bypassing Viterbi) GeoJSON: " + geoJson);
            }
            
            return result;
        }
        
        // Case 3: Fall back to Viterbi algorithm
        System.out.println("Case 3: Using Viterbi algorithm for map matching");
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

        // Compute bestPath node sequence: node[i] is departure of edge[i], node[i+1] is arrival.
        // We follow the actual traversal direction: for each edge, the arrival is whichever endpoint
        // is NOT the departure. This handles edges traversed in either physical direction.
        int[] bestPathNodes = new int[bestPathEdges.size() + 1];
        bestPathNodes[0] = bestPathEdges.get(0).getBaseNode();
        for (int i = 0; i < bestPathEdges.size(); i++) {
            EdgeIteratorState e = bestPathEdges.get(i);
            int from = bestPathNodes[i];
            bestPathNodes[i + 1] = (e.getBaseNode() == from) ? e.getAdjNode() : e.getBaseNode();
        }

        // Build merged path by replacing bestPath sections with routed segments
        List<EdgeIteratorState> mergedPath = new ArrayList<>();
        int bpCursor = 0; // edge index cursor

        for (int segNum = 0; segNum < perSegmentRoutedEdges.size(); segNum++) {
            int startNode = segmentBoundaryNodes.get(segNum)[0];
            int endNode = segmentBoundaryNodes.get(segNum)[1];

            // Skip unspliceable segments — bestPath continues as-is
            if (startNode == -1 || endNode == -1) {
                System.out.println("  Skipping unspliceable segment " + segNum);
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

            // Insert routed segment edges (already correctly oriented from Path.calcEdges())
            List<EdgeIteratorState> segmentEdges = perSegmentRoutedEdges.get(segNum);
            if (!segmentEdges.isEmpty()) {
                mergedPath.addAll(segmentEdges);
            }

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

        // Validate merged path connectivity (baseNode=departure, adjNode=arrival for calcEdges() output)
        if (mergedPath.size() > 1) {
            int prevNode = mergedPath.get(0).getBaseNode(); // baseNode = departure
            int discontinuities = 0;
            for (int i = 0; i < mergedPath.size(); i++) {
                EdgeIteratorState e = mergedPath.get(i);
                if (e.getBaseNode() == prevNode) {
                    prevNode = e.getAdjNode(); // advance to arrival
                } else if (e.getAdjNode() == prevNode) {
                    prevNode = e.getBaseNode();
                } else {
                    if (discontinuities < 5) {
                        System.out.println("WARNING: Merged path discontinuity at edge " + i +
                                ": expected node " + prevNode + " but edge " + e.getEdge() +
                                " connects " + e.getBaseNode() + "<->" + e.getAdjNode());
                    }
                    discontinuities++;
                    prevNode = e.getBaseNode();
                }
            }
            if (discontinuities > 0) {
                System.out.println("WARNING: Merged path has " + discontinuities + " discontinuities out of " + mergedPath.size() + " edges");
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

    /**
     * Calculate the total direct distance between consecutive observations.
     *
     * @param filteredObservations List of observations to calculate distance for
     * @return Total direct distance in meters
     */
    private double calculateTotalDirectDistance(List<Observation> filteredObservations) {
        if (filteredObservations.size() < 2) {
            return 0.0;
        }

        double totalDirectDistance = 0.0;
        for (int i = 0; i < filteredObservations.size() - 1; i++) {
            GHPoint fromPoint = new GHPoint(
                    filteredObservations.get(i).getPoint().lat,
                    filteredObservations.get(i).getPoint().lon);
            GHPoint toPoint = new GHPoint(
                    filteredObservations.get(i + 1).getPoint().lat,
                    filteredObservations.get(i + 1).getPoint().lon);

            double segmentDistance = DistanceCalcEarth.DIST_EARTH.calcDist(
                    fromPoint.lat, fromPoint.lon, toPoint.lat, toPoint.lon);
            totalDirectDistance += segmentDistance;
        }

        return totalDirectDistance;
    }

    /**
     * Calculate the maximum allowed distance for the path based on observations.
     * where directDistance is the sum of distances between consecutive observations.
     */
    private double calculateMaxAllowedDistance(List<Observation> filteredObservations) {
        if (filteredObservations.size() < 2) {
            return Double.MAX_VALUE; // No constraint for single observation
        }

        double totalDirectDistance = calculateTotalDirectDistance(filteredObservations);

        // Be less restrictive for short paths
        if (totalDirectDistance < 1500) return totalDirectDistance + 2000.0;
        return totalDirectDistance * 1.5;
    }

    /**
     * Result of path analysis containing the best path and its metadata
     */
    private static class PathAnalysisResult {
        final Path bestPath;
        final int bestPathIndex;
        final List<List<Snap>> bestPathSnaps;
        final boolean hasDirectPath;
        final Path directPath;
        final List<List<Snap>> directPathSnaps;

        PathAnalysisResult(Path bestPath, int bestPathIndex, List<List<Snap>> bestPathSnaps,
                          boolean hasDirectPath, Path directPath, List<List<Snap>> directPathSnaps) {
            this.bestPath = bestPath;
            this.bestPathIndex = bestPathIndex;
            this.bestPathSnaps = bestPathSnaps;
            this.hasDirectPath = hasDirectPath;
            this.directPath = directPath;
            this.directPathSnaps = directPathSnaps;
        }
    }

    /**
     * Analyzes routed paths to find the best path and check if any path contains all observations
     */
    private PathAnalysisResult analyzeRoutedPaths(List<Path> routedPaths,
                                                  List<Observation> filteredObservations,
                                                  List<List<Snap>> snapsPerObservationTmp,
                                                  SnappedPointsAnalyzer.AnalysisResult analysisResult,
                                                  boolean forceInitialRouting) {
        int[] snapsOnPathCounts = analysisResult.snapsOnPathCounts;
        List<List<Boolean>> snapsNotOnRoutedPaths = analysisResult.snapsNotOnRoutedPaths;
        List<Set<Integer>> routedPathsPathEdgeIndices = analysisResult.routedPathsPathEdgeIndices;
        List<List<List<Snap>>> snapsPerObservationOnRoutedPathTmpList = analysisResult.snapsPerObservationOnRoutedPathTmpList;

        int maxSnapsCount = -1;
        int bestPathIndex = -1;
        Path bestPath = null;
        List<List<Snap>> bestPathSnaps = new ArrayList<>();
        
        Path directPath = null;
        List<List<Snap>> directPathSnaps = new ArrayList<>();
        boolean hasDirectPath = false;
        double totalDirectDistance = calculateTotalDirectDistance(filteredObservations);
        System.out.println("Total direct distance: " + totalDirectDistance);

        for (int routedPathsIndex = 0; routedPathsIndex < routedPaths.size(); routedPathsIndex++) {
            Path tmpRoutedPath = routedPaths.get(routedPathsIndex);
            Set<Integer> pathEdgeIndices = routedPathsPathEdgeIndices.get(routedPathsIndex);
            
            if (!tmpRoutedPath.isFound()) {
                continue;
            }
            
            int snapsOnPathCount = snapsOnPathCounts[routedPathsIndex];

            System.out.println("Path #" + (routedPathsIndex + 1) + ": " + snapsOnPathCount + " snaps out of " +
                    snapsPerObservationTmp.size() + ", edges used: " + pathEdgeIndices.size() + ", path distance: " + tmpRoutedPath.getDistance() + "m.");

            if (pathEdgeIndices.size() <= 2)
                System.out.println("Path #" + (routedPathsIndex + 1) + ", all snaps on the first and last edges");

            int finalRoutedPathsIndex = routedPathsIndex;
            boolean allSnapsOnRoutedPath = snapsNotOnRoutedPaths.stream().noneMatch(snap -> snap.get(finalRoutedPathsIndex));

            // Track the path with the most snaps
            if (snapsOnPathCount > maxSnapsCount &&
                    (pathEdgeIndices.size() > 2 || filteredObservations.size() == 2 || tmpRoutedPath.getDistance() > totalDirectDistance)) {
                maxSnapsCount = snapsOnPathCount;
                bestPathIndex = routedPathsIndex;
                bestPath = tmpRoutedPath;
                bestPathSnaps = new ArrayList<>();
                List<List<Snap>> pathSnaps = snapsPerObservationOnRoutedPathTmpList.get(routedPathsIndex);
                for (int i = 0; i < filteredObservations.size(); i++) {
                    bestPathSnaps.add(new ArrayList<>());
                }
                for (List<Snap> pathSnap : pathSnaps) {
                    if (!pathSnap.isEmpty()) {
                        int queryPointIndex = pathSnap.get(0).getQueryPoint().index;
                        for (int j = 0; j < filteredObservations.size(); j++) {
                            if (filteredObservations.get(j).getPoint().index == queryPointIndex) {
                                bestPathSnaps.get(j).addAll(pathSnap);
                                break;
                            }
                        }
                    }
                }
            }

            // Check if all snaps are on this path (case 1)
            if (allSnapsOnRoutedPath && !forceInitialRouting ) {
                directPath = tmpRoutedPath;
                directPathSnaps = new ArrayList<>(snapsPerObservationOnRoutedPathTmpList.get(routedPathsIndex));
                hasDirectPath = true;
                System.out.println("All observations on the path #" + (finalRoutedPathsIndex + 1) + ": using direct routing for map matching");
                break;
            }
        }

        // Print selected path as GeoJSON
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

        return new PathAnalysisResult(bestPath, bestPathIndex, bestPathSnaps, hasDirectPath, directPath, directPathSnaps);
    }

    /**
     * Check if the processing time limit has been exceeded and throw an exception if so.
     */
    private void checkTimeLimit(StopWatch sw) {
        if (sw.getCurrentSeconds() >= maxProcessingTimeSeconds) {
            throw new IllegalArgumentException("Time limit of " + maxProcessingTimeSeconds + "s exceeded.");
        }
    }

    /**
     * Attempts via-waypoint routing through missing observations (case 2)
     */
    private MatchResult attemptViaWaypointRouting(Path bestPath, int bestPathIndex,
                                                  List<List<Snap>> bestPathSnaps,
                                                  List<Observation> filteredObservations,
                                                  List<List<Snap>> snapsPerObservationTmp,
                                                  List<List<Boolean>> snapsNotOnRoutedPaths,
                                                  List<Observation> observations,
                                                  StopWatch sw) {

        // Find observations not on the best path
        List<Integer> observationsNotOnBestPathIndices = new ArrayList<>();
        for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
            if (snapsNotOnRoutedPaths.get(observationsIndex).get(bestPathIndex)) {
                observationsNotOnBestPathIndices.add(observationsIndex);
            }
        }
        
        // Also find observations not on any path for comparison
        List<Integer> observationsNotOnAnyPathIndices = new ArrayList<>();
        for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
            List<Boolean> snapNotOnRoutedPath = snapsNotOnRoutedPaths.get(observationsIndex);
            List<Snap> snaps = snapsPerObservationTmp.get(observationsIndex);
            if (snapNotOnRoutedPath.stream().allMatch(Boolean::booleanValue) && !snaps.isEmpty()) {
                observationsNotOnAnyPathIndices.add(observationsIndex);
            }
        }
        
        if (observationsNotOnBestPathIndices.isEmpty()) {
            System.out.println("Via-waypoint routing: no off-path observations found");
            return null;
        }
        
        System.out.println("Attempting via-waypoint routing through " + observationsNotOnBestPathIndices.size() +
                " off-path observations (relative to best path #" + (bestPathIndex) + ")" +
                " (was " + observationsNotOnAnyPathIndices.size() + " off all paths)");
        
        // Build waypoint segments and attempt routing
        ViaWaypointRoutingResult routingResult = performViaWaypointRouting(
                observationsNotOnBestPathIndices, filteredObservations, 
                bestPath, bestPathSnaps, snapsPerObservationTmp, sw);
        
        if (!routingResult.success) {
            System.out.println("Via-waypoint routing: failed, falling back to Viterbi");
            statistics.put("usedViaWaypointRouting", false);
            return null;
        }
        
        List<EdgeIteratorState> bestPathEdges = bestPath.calcEdges();
        
        List<EdgeIteratorState> mergedPath = buildMergedEdgeList(
                bestPathEdges, routingResult.perSegmentRoutedEdges, 
                routingResult.segmentBoundaryNodes);
        
        System.out.println("Via-waypoint routing: successfully routed detour segments. Total merged edges=" + mergedPath.size());

        // Fail-early: ensure merged path distance respects the max allowed distance
        // (same constraint used for the direct path in Case 1). Otherwise fall back to Viterbi.
        double mergedDistance = 0.0;
        for (EdgeIteratorState e : mergedPath) mergedDistance += e.getDistance();
        double maxAllowedDistance = calculateMaxAllowedDistance(filteredObservations);
        if (mergedDistance > maxAllowedDistance) {
            System.out.println("Via-waypoint routing: merged path distance " +
                    String.format("%.0f", mergedDistance) + "m exceeds threshold " +
                    String.format("%.0f", maxAllowedDistance) + "m - falling back to Viterbi");
            statistics.put("usedViaWaypointRouting", false);
            return null;
        }

        // Debug: Check merged path edges before creating MapMatchedPath
        for (int i = 0; i < Math.min(5, mergedPath.size()); i++) {
            EdgeIteratorState edge = mergedPath.get(i);
            System.out.println("  Merged edge " + i + ": edgeId=" + edge.getEdge() + 
                    ", baseNode=" + edge.getBaseNode() + ", adjNode=" + edge.getAdjNode());
        }
        
        // Build edge matches
        List<EdgeMatch> edgeMatches = buildEdgeMatchesForMergedPath(
                mergedPath, observations, filteredObservations,
                bestPathSnaps, routingResult.routedPathSnaps);
        
        // Create and return result
        statistics.put("usedDirectRouting", false);
        statistics.put("forcedDirectRouting", false);
        statistics.put("usedViaWaypointRouting", true);
        statistics.put("visitedNodes", router.getVisitedNodes());
        
        processedUpTo = observations.size() - 1;
        
        printMergedPathGeoJson(mergedPath, observations.size(),
                edgeMatches.stream().mapToInt(em -> em.getStates().size()).sum());
        
        Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
        Path mergedMapMatchedPath = new MapMatchedPath(queryGraph, queryGraphWeighting, mergedPath);
        
        MatchResult result = new MatchResult(edgeMatches);
        result.setMergedPath(mergedMapMatchedPath);
        result.setMatchMillis(mergedMapMatchedPath.getTime());
        result.setMatchLength(mergedMapMatchedPath.getDistance());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(queryGraphWeighting);
        
        return result;
    }
    
    /**
     * Result of a single-leg routing attempt between two observations.
     */
    private static class LegResult {
        final Path path;
        final Snap fromSnap;
        final Snap toSnap;
        LegResult(Path path, Snap fromSnap, Snap toSnap) {
            this.path = path;
            this.fromSnap = fromSnap;
            this.toSnap = toSnap;
        }
    }

    /**
     * Mutable result of routing all legs within one off-path segment.
     * Fields are updated in place by salvage and splice steps.
     */
    private static class SegmentResult {
        final List<EdgeIteratorState> edges = new ArrayList<>();
        int startNode = -1;
        int endNode   = -1;
        boolean spliceable    = true;
        boolean allLegsRouted = true;
        Snap lastToSnap = null;  // to-snap of the last committed leg (used by salvage)
    }

    /**
     * Result of via-waypoint routing
     */
    private static class ViaWaypointRoutingResult {
        final boolean success;
        final List<List<EdgeIteratorState>> perSegmentRoutedEdges;
        final List<int[]> segmentBoundaryNodes;
        final List<List<Snap>> routedPathSnaps;
        
        ViaWaypointRoutingResult(boolean success, List<List<EdgeIteratorState>> perSegmentRoutedEdges,
                                List<int[]> segmentBoundaryNodes, List<List<Snap>> routedPathSnaps) {
            this.success = success;
            this.perSegmentRoutedEdges = perSegmentRoutedEdges;
            this.segmentBoundaryNodes = segmentBoundaryNodes;
            this.routedPathSnaps = routedPathSnaps;
        }
    }
    
    /**
     * Routes each contiguous off-path segment via chained waypoint legs and splices the results
     * back onto the best path.
     *
     * <p>Steps for each segment:
     * <ol>
     *   <li>Route all waypoint-to-waypoint legs ({@link #routeSegmentLegs}).</li>
     *   <li>Salvage a partial segment if the leg loop exits early ({@link #salvagePartialSegment}).</li>
     *   <li>Bridge segment boundaries onto the best path ({@link #spliceSegmentBoundaries}).</li>
     * </ol>
     */
    private ViaWaypointRoutingResult performViaWaypointRouting(
            List<Integer> observationsNotOnBestPathIndices,
            List<Observation> filteredObservations,
            Path bestPath,
            List<List<Snap>> bestPathSnaps,
            List<List<Snap>> snapsPerObservationTmp,
            StopWatch sw) {

        Set<Integer> offPathSet = new LinkedHashSet<>(observationsNotOnBestPathIndices);
        List<List<Integer>> offPathSegments       = computeOffPathSegments(observationsNotOnBestPathIndices);
        List<List<Integer>> segmentWaypointIndices = buildWaypointsList(filteredObservations, offPathSegments, offPathSet);

        if (!validateAndLogWaypointSnaps(segmentWaypointIndices, snapsPerObservationTmp, filteredObservations, offPathSet))
            return new ViaWaypointRoutingResult(false, null, null, null);

        Set<Integer> bestPathNodeSet = buildBestPathNodeSet(bestPath);

        List<List<EdgeIteratorState>> perSegmentRoutedEdges = new ArrayList<>();
        List<int[]>       segmentBoundaryNodes = new ArrayList<>();
        List<List<Snap>>  routedPathSnaps      = new ArrayList<>();
        for (int i = 0; i < filteredObservations.size(); i++) routedPathSnaps.add(new ArrayList<>());

        boolean allSegmentsRouted = true;
        for (int segNum = 0; segNum < segmentWaypointIndices.size(); segNum++) {
            checkTimeLimit(sw);
            List<Integer> waypoints = segmentWaypointIndices.get(segNum);

            SegmentResult result = routeSegmentLegs(waypoints, snapsPerObservationTmp,
                    filteredObservations, offPathSet, routedPathSnaps, segNum, sw);

            if (!result.spliceable) { result.startNode = -1; result.endNode = -1; }

            salvagePartialSegment(result, bestPathNodeSet);

            if (result.spliceable)
                spliceSegmentBoundaries(result, waypoints, bestPathNodeSet,
                        bestPathSnaps, filteredObservations, offPathSet, segNum);

            perSegmentRoutedEdges.add(result.edges);
            segmentBoundaryNodes.add(new int[]{result.startNode, result.endNode});
            logSegmentGeoJson(result.edges, segNum, waypoints.size(), result.spliceable, result.startNode, result.endNode);

            if (!result.allLegsRouted) { allSegmentsRouted = false; break; }
        }

        if (!allSegmentsRouted || perSegmentRoutedEdges.isEmpty())
            return new ViaWaypointRoutingResult(false, null, null, null);

        return new ViaWaypointRoutingResult(true, perSegmentRoutedEdges, segmentBoundaryNodes, routedPathSnaps);
    }

    /**
     * Returns the maximum acceptable route distance between two GPS points.
     * Allows up to 2× the direct great-circle distance, with a minimum slack of 2 km.
     */
    private static double routingThreshold(GHPoint from, GHPoint to) {
        double direct = DistanceCalcEarth.DIST_EARTH.calcDist(from.lat, from.lon, to.lat, to.lon);
        return Math.max(direct * 2.0, direct + 2000.0);
    }

    /**
     * Attempts to route from {@code fromNode} to {@code toNode} on the unified query graph.
     *
     * @return the shortest {@link Path} if reachable, or {@code null} if unreachable or routing throws.
     */
    private Path tryRoute(int fromNode, int toNode) {
        if (fromNode == toNode) return null;
        try {
            List<Path> paths = router.calcPaths(queryGraph, fromNode, EdgeIterator.ANY_EDGE,
                    new int[]{toNode}, new int[]{EdgeIterator.ANY_EDGE});
            return (!paths.isEmpty() && paths.get(0).isFound()) ? paths.get(0) : null;
        } catch (Exception ignored) {
            return null;
        }
    }

    /**
     * Returns the first {@code max} elements of {@code list}, or the full list if smaller.
     */
    private static <T> List<T> limit(List<T> list, int max) {
        return list.size() > max ? list.subList(0, max) : list;
    }

    /**
     * Builds the from-candidate list for the current leg.
     * When chaining (chainSnap != null), the chain snap is placed first so it is unconditionally
     * accepted by the bridge pre-filter; all other candidates follow.
     */
    private static List<Snap> buildChainedFromCandidates(List<Snap> allFromCandidates, Snap chainSnap) {
        if (chainSnap == null) return allFromCandidates;
        List<Snap> result = new ArrayList<>();
        result.add(chainSnap);
        for (Snap s : allFromCandidates)
            if (s.getClosestNode() != chainSnap.getClosestNode())
                result.add(s);
        return result;
    }

    /**
     * Searches all (from, to) snap combinations for the best leg path within {@code threshold}.
     * <p>
     * For non-chained from-snaps a bridge pre-check verifies that {@code chainNode} can reach the
     * candidate from-node within {@link #MAX_BRIDGE_DISTANCE}, preventing infeasible intra-bridges
     * from being chosen downstream.
     *
     * @param fromCandidates ordered from-snap candidates (chain snap first when chaining)
     * @param toCandidates   to-snap candidates
     * @param chainNode      closing node of the previous leg, or {@code -1} for the first leg
     * @param threshold      maximum acceptable path distance
     * @return the best {@link LegResult} within threshold, or {@code null} if none found
     */
    private LegResult findBestLeg(List<Snap> fromCandidates, List<Snap> toCandidates,
                                   int chainNode, double threshold) {
        LegResult best = null;
        outer:
        for (Snap fromSnap : fromCandidates) {
            int fromNode = fromSnap.getClosestNode();
            if (chainNode >= 0 && fromNode != chainNode) {
                Path bridge = tryRoute(chainNode, fromNode);
                if (bridge == null || bridge.getDistance() > MAX_BRIDGE_DISTANCE) continue;
            }
            for (Snap toSnap : toCandidates) {
                Path path = tryRoute(fromNode, toSnap.getClosestNode());
                if (path == null) continue;
                if (best == null || path.getDistance() < best.path.getDistance()) {
                    best = new LegResult(path, fromSnap, toSnap);
                    if (path.getDistance() <= threshold) break outer;
                }
            }
        }
        return (best != null && best.path.getDistance() <= threshold) ? best : null;
    }

    /**
     * Removes the last {@code count} edges from the given edge list.
     */
    private static void trimTail(List<EdgeIteratorState> edges, int count) {
        for (int i = 0; i < count; i++) edges.remove(edges.size() - 1);
    }

    /**
     * Builds the ordered list of waypoints for each off-path segment.
     * Each list begins with the closest on-path anchor before the segment (if any),
     * followed by the off-path observations, then the closest on-path anchor after.
     * <p>
     * All values are <em>filtered positions</em> — indices into {@code filteredObservations}.
     */
    @NotNull
    private static List<List<Integer>> buildWaypointsList(List<Observation> filteredObservations, List<List<Integer>> offPathSegments, Set<Integer> offPathSet) {
        List<List<Integer>> segmentWaypointIndices = new ArrayList<>();

        System.out.println("Via-waypoint routing: found " + offPathSegments.size() + " contiguous off-path segment(s)");
        for (int segNum = 0; segNum < offPathSegments.size(); segNum++) {
            List<Integer> seg = offPathSegments.get(segNum);

            // Find the closest on-path anchor before and after (filtered positions)
            Integer anchorBefore = null;
            Integer anchorAfter  = null;
            for (int i = seg.get(0) - 1; i >= 0; i--)
                if (!offPathSet.contains(i)) { anchorBefore = i; break; }
            for (int i = seg.get(seg.size() - 1) + 1; i < filteredObservations.size(); i++)
                if (!offPathSet.contains(i)) { anchorAfter = i; break; }

            System.out.println("  Segment " + segNum + ": off-path obs "
                    + filteredObservations.get(seg.get(0)).getPoint().index
                    + "-" + filteredObservations.get(seg.get(seg.size() - 1)).getPoint().index
                    + " (anchor before: obs " + (anchorBefore != null ? filteredObservations.get(anchorBefore).getPoint().index : "NONE")
                    + ", anchor after: obs "  + (anchorAfter  != null ? filteredObservations.get(anchorAfter).getPoint().index  : "NONE") + ")");

            List<Integer> waypoints = new ArrayList<>();
            if (anchorBefore != null) waypoints.add(anchorBefore);
            waypoints.addAll(seg);
            if (anchorAfter  != null) waypoints.add(anchorAfter);

            segmentWaypointIndices.add(waypoints);
        }
        return segmentWaypointIndices;
    }

    @NotNull
    private static List<List<Integer>> computeOffPathSegments(List<Integer> observationsNotOnBestPathIndices) {
        List<List<Integer>> offPathSegments = new ArrayList<>();
        List<Integer> currentSegment = null;
        for (int idx : observationsNotOnBestPathIndices) {
            if (currentSegment == null || idx != currentSegment.get(currentSegment.size() - 1) + 1) {
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
        return offPathSegments;
    }

    /**
     * Validates that every waypoint across all segments has at least one snap candidate.
     * Also logs snap details for each unique waypoint position.
     *
     * @return {@code true} if all waypoints have snaps; {@code false} if any is missing (routing cannot proceed)
     */
    private static boolean validateAndLogWaypointSnaps(
            List<List<Integer>> segmentWaypointIndices,
            List<List<Snap>> snapsPerObservationTmp,
            List<Observation> filteredObservations,
            Set<Integer> offPathSet) {
        Set<Integer> logged = new HashSet<>();
        for (List<Integer> waypoints : segmentWaypointIndices) {
            for (int filteredPos : waypoints) {
                if (!logged.add(filteredPos)) continue;
                List<Snap> candidateSnaps = snapsPerObservationTmp.get(filteredPos);
                if (candidateSnaps == null || candidateSnaps.isEmpty()) {
                    System.out.println("Via-waypoint routing: some waypoints have no snaps");
                    return false;
                }
                Observation obs = filteredObservations.get(filteredPos);
                Snap closestSnap = candidateSnaps.get(0);
                boolean isOffPath = offPathSet.contains(filteredPos);
                System.out.println("  Obs " + obs.getPoint().index + (isOffPath ? " [OFF-PATH]" : " [ON-PATH anchor]") +
                        ": GPS=" + obs.getPoint().lat + "," + obs.getPoint().lon +
                        " -> snapped to node " + closestSnap.getClosestNode() +
                        " at " + closestSnap.getSnappedPoint().lat + "," + closestSnap.getSnappedPoint().lon +
                        " on edge " + closestSnap.getClosestEdge().getEdge() +
                        " (name=" + closestSnap.getClosestEdge().getName() + ")" +
                        " dist=" + String.format("%.1f", closestSnap.getQueryDistance()) + "m" +
                        " (" + candidateSnaps.size() + " candidates)");
            }
        }
        return true;
    }

    /**
     * Extracts the set of all graph nodes visited by the given best path.
     */
    private static Set<Integer> buildBestPathNodeSet(Path bestPath) {
        List<EdgeIteratorState> edges = bestPath.calcEdges();
        Set<Integer> nodeSet = new HashSet<>();
        if (edges.isEmpty()) return nodeSet;
        int prev = edges.get(0).getBaseNode();
        nodeSet.add(prev);
        for (EdgeIteratorState e : edges) {
            int next = (e.getBaseNode() == prev) ? e.getAdjNode() : e.getBaseNode();
            nodeSet.add(next);
            prev = next;
        }
        return nodeSet;
    }

    /**
     * Routes all waypoint-to-waypoint legs for a single off-path segment.
     * Applies four fallbacks in order: waypoint-skip, prev-leg snap fix (L1/L2),
     * short-spur undo, and anchor back-step. Updates {@code routedPathSnaps} in place.
     *
     * @return a {@link SegmentResult} with the committed edges and boundary nodes
     */
    private SegmentResult routeSegmentLegs(
            List<Integer> waypoints,
            List<List<Snap>> snapsPerObservationTmp,
            List<Observation> filteredObservations,
            Set<Integer> offPathSet,
            List<List<Snap>> routedPathSnaps,
            int segNum,
            StopWatch sw) {

        SegmentResult result = new SegmentResult();
        Snap previousLegToSnap = null;
        Snap prevChainSnap         = null;
        Snap prevPrevChainSnap     = null;
        int  lastIterEdgeCount     = 0;
        int  prevPrevIterEdgeCount = 0;
        double lastLegRouteDist    = 0;
        int  anchorBackSteps       = 0;
        int  wpIdx                 = 0;

        while (wpIdx < waypoints.size() - 1) {
            checkTimeLimit(sw);

            int edgesAtIterStart = result.edges.size();
            int fromFilteredPos  = waypoints.get(wpIdx);
            int toFilteredPos    = waypoints.get(wpIdx + 1);

            List<Snap> allFromCandidates = limit(snapsPerObservationTmp.get(fromFilteredPos), MAX_SNAP_CANDIDATES);
            List<Snap> toCandidates      = limit(snapsPerObservationTmp.get(toFilteredPos),   MAX_SNAP_CANDIDATES);
            List<Snap> fromCandidates    = buildChainedFromCandidates(allFromCandidates, previousLegToSnap);
            double threshold             = routingThreshold(fromCandidates.get(0).getQueryPoint(),
                                                            toCandidates.get(0).getQueryPoint());
            int chainNode                = (previousLegToSnap != null) ? previousLegToSnap.getClosestNode() : -1;

            // --- Primary routing: best (from, to) snap pair within threshold.
            // Non-chained from-snaps are bridge-pre-filtered to prevent infeasible intra-bridges. ---
            LegResult leg = findBestLeg(fromCandidates, toCandidates, chainNode, threshold);

            // --- Fallback 1: Waypoint-skip ---
            // Route the chain directly to waypoints[wpIdx+2], bypassing the unreachable waypoints[wpIdx+1].
            if (leg == null && previousLegToSnap != null && wpIdx < waypoints.size() - 2) {
                int skipToFilteredPos     = waypoints.get(wpIdx + 2);
                List<Snap> skipCandidates = limit(snapsPerObservationTmp.get(skipToFilteredPos), MAX_SNAP_CANDIDATES);
                if (!skipCandidates.isEmpty()) {
                    double skipThreshold = routingThreshold(
                            previousLegToSnap.getQueryPoint(), skipCandidates.get(0).getQueryPoint());
                    for (Snap toSnap : skipCandidates) {
                        Path p = tryRoute(previousLegToSnap.getClosestNode(), toSnap.getClosestNode());
                        if (p != null && p.getDistance() <= skipThreshold) {
                            System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] waypoint-skip obs "
                                    + filteredObservations.get(toFilteredPos).getPoint().index
                                    + " -> routing chain to obs "
                                    + filteredObservations.get(skipToFilteredPos).getPoint().index);
                            leg           = new LegResult(p, previousLegToSnap, toSnap);
                            wpIdx++;
                            toFilteredPos = skipToFilteredPos;
                            break;
                        }
                    }
                }
            }

            // --- Fallback 2: Prev-leg snap fix (L1 / L2) ---
            // The bridge pre-filter may have locked all from-snaps to a dead-end node.
            // Scan ALL from-snap candidates (ignoring the pre-filter) for one that reaches the next obs.
            // If found (altLeg), re-route the last 1 leg (L1) or 2 legs (L2) to reach that better from-snap.
            if (leg == null && prevChainSnap != null && lastIterEdgeCount > 0) {
                LegResult altLeg = null;
                outer:
                for (Snap cand : snapsPerObservationTmp.get(fromFilteredPos)) {
                    if (cand.getClosestNode() == previousLegToSnap.getClosestNode()) continue;
                    for (Snap toSnap : toCandidates) {
                        Path p = tryRoute(cand.getClosestNode(), toSnap.getClosestNode());
                        if (p != null && p.getDistance() <= threshold) {
                            altLeg = new LegResult(p, cand, toSnap);
                            break outer;
                        }
                    }
                }
                if (altLeg != null) {
                    int altFromNode = altLeg.fromSnap.getClosestNode();
                    List<Snap> prevLegFromTrials = new ArrayList<>();
                    prevLegFromTrials.add(prevChainSnap);
                    List<Snap> prevLegAllCands = snapsPerObservationTmp.get(waypoints.get(wpIdx - 1));
                    if (prevLegAllCands != null)
                        for (Snap s : prevLegAllCands)
                            if (s.getClosestNode() != prevChainSnap.getClosestNode())
                                prevLegFromTrials.add(s);
                    for (Snap prevLegTrial : prevLegFromTrials) {
                        int prevLegNode = prevLegTrial.getClosestNode();
                        Path prevLegPath = tryRoute(prevLegNode, altFromNode);
                        if (prevLegPath == null || prevLegPath.getDistance() >
                                routingThreshold(prevLegTrial.getQueryPoint(), altLeg.fromSnap.getQueryPoint())) continue;
                        if (prevLegNode == prevChainSnap.getClosestNode()) {
                            // L1: replace last leg only
                            trimTail(result.edges, lastIterEdgeCount);
                            result.edges.addAll(prevLegPath.calcEdges());
                            edgesAtIterStart  = result.edges.size();
                            previousLegToSnap = altLeg.fromSnap;
                            chainNode         = altLeg.fromSnap.getClosestNode();
                            leg               = altLeg;
                            System.out.println("  [Seg " + segNum + " leg " + wpIdx
                                    + "] prev-leg snap fix L1: " + prevLegNode + "->" + altFromNode
                                    + " (" + String.format("%.0f", prevLegPath.getDistance()) + "m)");
                            break;
                        } else if (prevPrevChainSnap != null && prevPrevIterEdgeCount > 0) {
                            // L2: replace last two legs
                            Path ppPath = tryRoute(prevPrevChainSnap.getClosestNode(), prevLegNode);
                            if (ppPath == null || ppPath.getDistance() >
                                    routingThreshold(prevPrevChainSnap.getQueryPoint(), prevLegTrial.getQueryPoint())) continue;
                            trimTail(result.edges, lastIterEdgeCount + prevPrevIterEdgeCount);
                            result.edges.addAll(ppPath.calcEdges());
                            result.edges.addAll(prevLegPath.calcEdges());
                            edgesAtIterStart  = result.edges.size();
                            previousLegToSnap = altLeg.fromSnap;
                            chainNode         = altLeg.fromSnap.getClosestNode();
                            leg               = altLeg;
                            System.out.println("  [Seg " + segNum + " leg " + wpIdx
                                    + "] prev-leg snap fix L2: " + prevPrevChainSnap.getClosestNode()
                                    + "->" + prevLegNode + "->" + altFromNode
                                    + " (" + String.format("%.0f", ppPath.getDistance()) + "m + "
                                    + String.format("%.0f", prevLegPath.getDistance()) + "m)");
                            break;
                        }
                    }
                }
            }

            // --- Fallback 3: Short-spur undo ---
            // If the previous leg was very short (<= 500m) it likely landed on a dead-end spur.
            // Roll it back and retry the current leg from the earlier chain node.
            if (leg == null && lastLegRouteDist > 0 && lastLegRouteDist <= 500.0
                    && lastIterEdgeCount > 0 && prevChainSnap != null
                    && prevChainSnap.getClosestNode() != previousLegToSnap.getClosestNode()) {
                System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] short-spur undo: removing "
                        + lastIterEdgeCount + " edges (last leg dist=" + String.format("%.0f", lastLegRouteDist)
                        + "m), retrying from node " + prevChainSnap.getClosestNode());
                trimTail(result.edges, lastIterEdgeCount);
                previousLegToSnap     = prevChainSnap;
                prevChainSnap         = null;
                prevPrevChainSnap     = null;
                lastIterEdgeCount     = 0;
                prevPrevIterEdgeCount = 0;
                lastLegRouteDist      = 0;
                continue;
            }

            // --- Fallback 4: Anchor back-step ---
            // If the first leg still fails, step back to the previous on-path observation and retry.
            if (leg == null && wpIdx == 0 && anchorBackSteps < MAX_ANCHOR_BACK_STEPS) {
                boolean anchorUpdated = false;
                for (int wb = fromFilteredPos - 1; wb >= 0; wb--) {
                    if (offPathSet.contains(wb)) continue;
                    if (!snapsPerObservationTmp.get(wb).isEmpty()) {
                        System.out.println("  [Seg " + segNum + " leg 0] anchor back-step "
                                + (anchorBackSteps + 1) + ": obs "
                                + filteredObservations.get(fromFilteredPos).getPoint().index
                                + " -> obs " + filteredObservations.get(wb).getPoint().index);
                        waypoints.set(0, wb);
                        anchorBackSteps++;
                        anchorUpdated = true;
                        break;
                    }
                }
                if (anchorUpdated) continue;
            }

            // All fallbacks exhausted
            if (leg == null) {
                System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] no suitable path (obs "
                        + filteredObservations.get(fromFilteredPos).getPoint().index
                        + " -> " + filteredObservations.get(toFilteredPos).getPoint().index
                        + "), threshold=" + String.format("%.0f", threshold) + "m");
                result.allLegsRouted = false;
                break;
            }

            // --- Commit leg ---
            boolean usedChainedSnap = (chainNode >= 0 && leg.fromSnap.getClosestNode() == chainNode);
            System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] obs "
                    + filteredObservations.get(fromFilteredPos).getPoint().index
                    + " -> " + filteredObservations.get(toFilteredPos).getPoint().index
                    + ": from=" + leg.fromSnap.getClosestNode()
                    + (usedChainedSnap ? "(chained)" : "(alt)") + " to=" + leg.toSnap.getClosestNode()
                    + " dist=" + String.format("%.0f", leg.path.getDistance()) + "m");

            if (wpIdx == 0)                    result.startNode = leg.fromSnap.getClosestNode();
            if (wpIdx == waypoints.size() - 2) result.endNode   = leg.toSnap.getClosestNode();

            routedPathSnaps.get(fromFilteredPos).add(leg.fromSnap);
            routedPathSnaps.get(toFilteredPos).add(leg.toSnap);

            // Insert intra-bridge if the chosen from-snap differs from the previous leg's end node
            if (chainNode >= 0 && leg.fromSnap.getClosestNode() != chainNode) {
                Path bridge = tryRoute(chainNode, leg.fromSnap.getClosestNode());
                if (bridge != null && bridge.getDistance() <= MAX_BRIDGE_DISTANCE) {
                    result.edges.addAll(bridge.calcEdges());
                    System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] intra-bridge OK: "
                            + chainNode + " -> " + leg.fromSnap.getClosestNode()
                            + " dist=" + String.format("%.0f", bridge.getDistance()) + "m");
                } else {
                    double bridgeDist = (bridge != null) ? bridge.getDistance() : -1;
                    System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] intra-bridge FAIL: "
                            + chainNode + " -> " + leg.fromSnap.getClosestNode()
                            + " dist=" + String.format("%.0f", bridgeDist) + "m (limit=" + MAX_BRIDGE_DISTANCE + ")");
                    result.spliceable = false;
                }
                if (!result.spliceable) break;
            }

            result.edges.addAll(leg.path.calcEdges());
            prevPrevChainSnap     = prevChainSnap;
            prevPrevIterEdgeCount = lastIterEdgeCount;
            prevChainSnap         = previousLegToSnap;
            previousLegToSnap     = leg.toSnap;
            lastIterEdgeCount     = result.edges.size() - edgesAtIterStart;
            lastLegRouteDist      = leg.path.getDistance();
            result.lastToSnap     = leg.toSnap;
            wpIdx++;
        }

        return result;
    }

    /**
     * Attempts to recover a valid segment end node when the leg loop exits before reaching
     * the final waypoint. Scans the committed edges for the last node that is also on the
     * best path, trims the segment there, and sets {@code result.endNode}.
     */
    private static void salvagePartialSegment(SegmentResult result, Set<Integer> bestPathNodeSet) {
        if (!result.spliceable || result.edges.isEmpty()
                || result.endNode >= 0 || result.lastToSnap == null) return;

        int candidateEndNode = result.lastToSnap.getClosestNode();
        if (bestPathNodeSet.contains(candidateEndNode)) {
            result.endNode = candidateEndNode;
            return;
        }

        // Scan for the last edge arrival that lands on the best path
        int prevNode   = result.edges.get(0).getBaseNode();
        int foundNode  = -1;
        int trimToEdge = -1;
        for (int e = 0; e < result.edges.size(); e++) {
            EdgeIteratorState edge = result.edges.get(e);
            int arrival = (edge.getBaseNode() == prevNode) ? edge.getAdjNode() : edge.getBaseNode();
            if (bestPathNodeSet.contains(arrival)) {
                foundNode  = arrival;
                trimToEdge = e;
            }
            prevNode = arrival;
        }

        if (foundNode >= 0) {
            while (result.edges.size() > trimToEdge + 1) result.edges.remove(result.edges.size() - 1);
            result.endNode = foundNode;
            System.out.println("  Segment end trimmed: last bestPath node in segment=" + result.endNode + " (edge " + trimToEdge + ")");
        } else {
            // Use chain end as a hint for the walk-forward splice to bridge from
            result.endNode = candidateEndNode;
            StringBuilder dbg = new StringBuilder("  Segment end candidate=" + result.endNode + " (not in bestPath). Last 5 arrivals: ");
            int prevNode2 = result.edges.get(0).getBaseNode();
            List<Integer> arrivals = new ArrayList<>();
            for (EdgeIteratorState edge : result.edges) {
                int arr = (edge.getBaseNode() == prevNode2) ? edge.getAdjNode() : edge.getBaseNode();
                arrivals.add(arr);
                prevNode2 = arr;
            }
            for (int i = Math.max(0, arrivals.size() - 5); i < arrivals.size(); i++)
                dbg.append(arrivals.get(i)).append("(bp=").append(bestPathNodeSet.contains(arrivals.get(i))).append(") ");
            System.out.println(dbg);
        }
    }

    /**
     * Bridges the segment's start and end nodes back onto the best path using
     * a walk-back (for start) and walk-forward (for end) scan over on-path snaps.
     * Updates {@code result.startNode} and {@code result.endNode} in place.
     * Sets them to -1 if no suitable bridge is found.
     */
    private void spliceSegmentBoundaries(
            SegmentResult result,
            List<Integer> waypoints,
            Set<Integer> bestPathNodeSet,
            List<List<Snap>> bestPathSnaps,
            List<Observation> filteredObservations,
            Set<Integer> offPathSet,
            int segNum) {

        // Start anchor walk-back
        if (result.startNode >= 0 && !bestPathNodeSet.contains(result.startNode)) {
            int anchorFiltPos = waypoints.get(0);
            GHPoint startNodePoint = filteredObservations.get(anchorFiltPos).getPoint();
            boolean spliceFound = false;
            for (int wb = anchorFiltPos; wb >= 0 && !spliceFound; wb--) {
                if (offPathSet.contains(wb)) continue;
                List<Snap> bpSnaps = bestPathSnaps.get(wb);
                if (bpSnaps.isEmpty()) continue;
                for (Snap bpSnap : bpSnaps) {
                    int spliceNode = bpSnap.getClosestNode();
                    if (!bestPathNodeSet.contains(spliceNode)) continue;
                    try {
                        List<Path> bridge = router.calcPaths(queryGraph, spliceNode, EdgeIterator.ANY_EDGE,
                                new int[]{result.startNode}, new int[]{EdgeIterator.ANY_EDGE});
                        double spliceThreshold = routingThreshold(bpSnap.getQueryPoint(), startNodePoint);
                        if (!bridge.isEmpty() && bridge.get(0).isFound()
                                && bridge.get(0).getDistance() <= spliceThreshold) {
                            List<EdgeIteratorState> bridgeEdges = bridge.get(0).calcEdges();
                            result.edges.addAll(0, bridgeEdges);
                            System.out.println("  Walk-back splice (start): obs " +
                                    filteredObservations.get(wb).getPoint().index +
                                    " -> segment start, bridge=" + bridgeEdges.size() +
                                    " edges, " + String.format("%.0f", bridge.get(0).getDistance()) + "m" +
                                    " (threshold=" + String.format("%.0f", spliceThreshold) + "m)");
                            result.startNode = spliceNode;
                            spliceFound = true;
                            break;
                        }
                    } catch (Exception e) { /* skip */ }
                }
            }
            if (!spliceFound) {
                System.out.println("  WARNING: could not find walk-back splice for start of segment " + segNum);
                result.startNode = -1;
            }
        }

        // End anchor walk-forward
        if (result.endNode >= 0 && !bestPathNodeSet.contains(result.endNode)) {
            int anchorFiltPos = waypoints.get(waypoints.size() - 1);
            GHPoint endNodePoint = filteredObservations.get(anchorFiltPos).getPoint();
            boolean spliceFound = false;
            for (int wf = anchorFiltPos; wf < filteredObservations.size() && !spliceFound; wf++) {
                if (offPathSet.contains(wf)) continue;
                List<Snap> bpSnaps = bestPathSnaps.get(wf);
                if (bpSnaps.isEmpty()) continue;
                for (Snap bpSnap : bpSnaps) {
                    int spliceNode = bpSnap.getClosestNode();
                    if (!bestPathNodeSet.contains(spliceNode)) continue;
                    try {
                        List<Path> bridge = router.calcPaths(queryGraph, result.endNode, EdgeIterator.ANY_EDGE,
                                new int[]{spliceNode}, new int[]{EdgeIterator.ANY_EDGE});
                        double spliceThreshold = routingThreshold(endNodePoint, bpSnap.getQueryPoint());
                        if (!bridge.isEmpty() && bridge.get(0).isFound()
                                && bridge.get(0).getDistance() <= spliceThreshold) {
                            List<EdgeIteratorState> bridgeEdges = bridge.get(0).calcEdges();
                            result.edges.addAll(bridgeEdges);
                            System.out.println("  Walk-forward splice (end): segment end -> obs " +
                                    filteredObservations.get(wf).getPoint().index +
                                    ", bridge=" + bridgeEdges.size() +
                                    " edges, " + String.format("%.0f", bridge.get(0).getDistance()) + "m" +
                                    " (threshold=" + String.format("%.0f", spliceThreshold) + "m)");
                            result.endNode = spliceNode;
                            spliceFound = true;
                            break;
                        }
                    } catch (Exception e) { /* skip */ }
                }
            }
            if (!spliceFound) {
                System.out.println("  WARNING: could not find walk-forward splice for end of segment " + segNum);
                result.endNode = -1;
            }
        }
    }

    /**
     * Prints a GeoJSON Feature representing the routed segment for external debug visualisation.
     */
    private static void logSegmentGeoJson(
            List<EdgeIteratorState> edges,
            int segNum,
            int waypointCount,
            boolean spliceable,
            int startNode,
            int endNode) {
        if (edges.isEmpty()) return;
        StringBuilder sb = new StringBuilder();
        sb.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        boolean first = true;
        for (EdgeIteratorState edge : edges) {
            PointList pts = edge.fetchWayGeometry(FetchMode.ALL);
            for (int i = 0; i < pts.size(); i++) {
                if (!first) sb.append(",");
                sb.append("[").append(pts.getLon(i)).append(",").append(pts.getLat(i)).append("]");
                first = false;
            }
        }
        double distance = edges.stream().mapToDouble(EdgeIteratorState::getDistance).sum();
        sb.append("]},\"properties\":{\"stroke\":\"#ff9900\",\"path_type\":\"via_waypoint_segment\",\"segment_index\":")
                .append(segNum)
                .append(",\"edges\":").append(edges.size())
                .append(",\"distance\":").append(distance)
                .append(",\"waypoints\":").append(waypointCount)
                .append(",\"spliceable\":").append(spliceable)
                .append(",\"start_node\":").append(startNode)
                .append(",\"end_node\":").append(endNode)
                .append("}}");
        System.out.println("Via-waypoint Segment #" + segNum + " GeoJSON: " + sb);
    }
}