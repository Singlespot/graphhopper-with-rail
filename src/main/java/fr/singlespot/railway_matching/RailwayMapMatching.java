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
        if (routedPath != null && pathAnalysis != null && pathAnalysis.hasDirectPath) {
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
     * Performs the actual via-waypoint routing through off-path segments
     */
    private ViaWaypointRoutingResult performViaWaypointRouting(
            List<Integer> observationsNotOnBestPathIndices,
            List<Observation> filteredObservations,
            Path bestPath,
            List<List<Snap>> bestPathSnaps,
            List<List<Snap>> snapsPerObservationTmp,
            StopWatch sw) {
        
        Set<Integer> offPathSet = new LinkedHashSet<>(observationsNotOnBestPathIndices);
        
        // Identify contiguous off-path segments
        List<List<Integer>> offPathSegments = computeOffPathSegments(observationsNotOnBestPathIndices);

        // Build waypoint lists for each segment
        List<List<Integer>> segmentWaypointIndices = buildWaypointsList(filteredObservations, offPathSegments, offPathSet);

        // Create mapping from original to filtered positions for debug output
        Map<Integer, Integer> originalToFilteredPos = new HashMap<>();
        for (int i = 0; i < filteredObservations.size(); i++) {
            originalToFilteredPos.put(filteredObservations.get(i).getPoint().index, i);
        }

        // Check waypoints have snaps
        Map<Integer, List<Snap>> waypointAllSnapsMap = new LinkedHashMap<>();
        for (List<Integer> waypoints : segmentWaypointIndices) {
            for (int obsIdx : waypoints) {
                if (waypointAllSnapsMap.containsKey(obsIdx)) continue;
                
                List<Snap> candidateSnaps = null;
                for (List<Snap> snapList : snapsPerObservationTmp) {
                    if (!snapList.isEmpty() && snapList.get(0).getQueryPoint().index == obsIdx) {
                        candidateSnaps = snapList;
                        break;
                    }
                }
                
                if (candidateSnaps == null || candidateSnaps.isEmpty()) {
                    System.out.println("Via-waypoint routing: some waypoints have no snaps");
                    return new ViaWaypointRoutingResult(false, null, null, null);
                }
                
                waypointAllSnapsMap.put(obsIdx, candidateSnaps);
                
                // Find the observation for this snap
                Observation obs = null;
                for (Observation filteredObs : filteredObservations) {
                    if (filteredObs.getPoint().index == obsIdx) {
                        obs = filteredObs;
                        break;
                    }
                }
                
                if (obs != null) {
                    Snap closestSnap = candidateSnaps.get(0);
                    // Get the filtered position to check offPathSet correctly
                    Integer filteredPos = originalToFilteredPos.get(obsIdx);
                    boolean isOffPath = filteredPos != null && offPathSet.contains(filteredPos);
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
        }
        
        // Route each segment
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
        
        List<List<EdgeIteratorState>> perSegmentRoutedEdges = new ArrayList<>();
        List<int[]> segmentBoundaryNodes = new ArrayList<>();
        List<List<Snap>> routedPathSnaps = new ArrayList<>();
        for (int i = 0; i < filteredObservations.size(); i++) {
            routedPathSnaps.add(new ArrayList<>());
        }
        boolean allSegmentsRouted = true;
        for (int segNum = 0; segNum < segmentWaypointIndices.size(); segNum++) {
            // Check if we're exceeding time limit
            checkTimeLimit(sw);
            
            List<Integer> waypoints = segmentWaypointIndices.get(segNum);
            List<EdgeIteratorState> segmentEdges = new ArrayList<>();
            int segmentStartNode = -1;
            int segmentEndNode = -1;
            Snap previousLegToSnap = null;
            boolean segmentSpliceable = true;
            
            final double MAX_BRIDGE_DISTANCE = 20000;
            Snap prevChainSnap = null;          // chain snap from 2 legs back
            int lastIterEdgeCount = 0;           // edges committed in the last iteration
            double lastLegRouteDist = 0;         // route distance of the last committed leg
            int wpIdx = 0;
            while (wpIdx < waypoints.size() - 1) {
                // Check time limit in inner loop as well
                checkTimeLimit(sw);
                
                int edgesAtIterStart = segmentEdges.size();
                int fromOriginalIdx = waypoints.get(wpIdx);
                int toOriginalIdx = waypoints.get(wpIdx + 1);
                List<Snap> allFromCandidates = waypointAllSnapsMap.get(fromOriginalIdx);
                List<Snap> toCandidates = waypointAllSnapsMap.get(toOriginalIdx);
                
                // Limit candidates
                int maxCandidates = 10;
                if (allFromCandidates.size() > maxCandidates)
                    allFromCandidates = allFromCandidates.subList(0, maxCandidates);
                if (toCandidates.size() > maxCandidates)
                    toCandidates = toCandidates.subList(0, maxCandidates);
                
                // Build from-candidate list with chaining
                List<Snap> fromCandidates;
                if (wpIdx > 0 && previousLegToSnap != null) {
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
                
                // Calculate suitable distance threshold
                GHPoint fromPoint = fromCandidates.get(0).getQueryPoint();
                GHPoint toPoint = toCandidates.get(0).getQueryPoint();
                double directDistance = DistanceCalcEarth.DIST_EARTH.calcDist(
                        fromPoint.lat, fromPoint.lon, toPoint.lat, toPoint.lon);
                double suitableDistanceThreshold = Math.max(directDistance * 2.0, directDistance + 2000.0);
                
                // Find best path (with bridge pre-filtering for alt from-snaps)
                Path bestLegPath = null;
                Snap bestFromSnap = null;
                Snap bestToSnap = null;
                boolean suitablePathFound = false;
                
                for (Snap fromSnap : fromCandidates) {
                    int fromNode = fromSnap.getClosestNode();
                    
                    // Pre-check bridge feasibility for non-chained snaps to avoid choosing
                    // a from-snap that would require an infeasible intra-bridge later
                    if (wpIdx > 0 && previousLegToSnap != null
                            && fromNode != previousLegToSnap.getClosestNode()) {
                        boolean bridgeFeasible = false;
                        try {
                            List<Path> bridgeCheck = router.calcPaths(queryGraph,
                                    previousLegToSnap.getClosestNode(), EdgeIterator.ANY_EDGE,
                                    new int[]{fromNode}, new int[]{EdgeIterator.ANY_EDGE});
                            bridgeFeasible = !bridgeCheck.isEmpty() && bridgeCheck.get(0).isFound()
                                    && bridgeCheck.get(0).getDistance() <= MAX_BRIDGE_DISTANCE;
                        } catch (Exception ignored) {}
                        if (!bridgeFeasible) continue;
                    }
                    
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
                
                // Waypoint-skip fallback: if no suitable path found with any bridgeable from-snap,
                // try routing from the chain directly to the next-next waypoint (bypass current to-waypoint)
                if (!suitablePathFound && previousLegToSnap != null && wpIdx < waypoints.size() - 2) {
                    int skipToOriginalIdx = waypoints.get(wpIdx + 2);
                    List<Snap> skipToCandidates = waypointAllSnapsMap.get(skipToOriginalIdx);
                    if (skipToCandidates != null) {
                        if (skipToCandidates.size() > maxCandidates)
                            skipToCandidates = skipToCandidates.subList(0, maxCandidates);
                        int chainNode = previousLegToSnap.getClosestNode();
                        GHPoint chainPoint = previousLegToSnap.getQueryPoint();
                        GHPoint skipToPoint = skipToCandidates.get(0).getQueryPoint();
                        double skipDirectDist = DistanceCalcEarth.DIST_EARTH.calcDist(
                                chainPoint.lat, chainPoint.lon, skipToPoint.lat, skipToPoint.lon);
                        double skipThreshold = Math.max(skipDirectDist * 2.0, skipDirectDist + 2000.0);
                        for (Snap toSnap : skipToCandidates) {
                            int toNode = toSnap.getClosestNode();
                            if (chainNode == toNode) continue;
                            try {
                                List<Path> skipPaths = router.calcPaths(queryGraph, chainNode, EdgeIterator.ANY_EDGE,
                                        new int[]{toNode}, new int[]{EdgeIterator.ANY_EDGE});
                                if (!skipPaths.isEmpty() && skipPaths.get(0).isFound()
                                        && skipPaths.get(0).getDistance() <= skipThreshold) {
                                    bestLegPath = skipPaths.get(0);
                                    bestFromSnap = previousLegToSnap;
                                    bestToSnap = toSnap;
                                    suitablePathFound = true;
                                    System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] waypoint-skip obs " + toOriginalIdx + " -> routing chain to obs " + skipToOriginalIdx);
                                    break;
                                }
                            } catch (Exception ignored) {}
                        }
                        if (suitablePathFound) {
                            wpIdx++; // skip the failed to-waypoint
                            toOriginalIdx = skipToOriginalIdx;
                        }
                    }
                }
                
                // Short-spur undo: if no path found and the last committed leg was very short
                // (chain landed at a dead-end spur), undo it and retry from the earlier chain
                if (!suitablePathFound && lastLegRouteDist > 0 && lastLegRouteDist <= 500.0
                        && lastIterEdgeCount > 0 && prevChainSnap != null
                        && prevChainSnap.getClosestNode() != previousLegToSnap.getClosestNode()) {
                    System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] short-spur undo: removing " + lastIterEdgeCount + " edges (last leg dist=" + String.format("%.0f", lastLegRouteDist) + "m), retrying from node " + prevChainSnap.getClosestNode());
                    for (int r = 0; r < lastIterEdgeCount; r++) segmentEdges.remove(segmentEdges.size() - 1);
                    previousLegToSnap = prevChainSnap;
                    prevChainSnap = null;       // prevent double-undo
                    lastIterEdgeCount = 0;
                    lastLegRouteDist = 0;
                    continue;                   // retry this wpIdx with updated chain
                }
                
                if (!suitablePathFound) {
                    System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] no suitable path (obs " + fromOriginalIdx + " -> " + toOriginalIdx + "), dist=" + (bestLegPath != null ? String.format("%.0f", bestLegPath.getDistance()) : "null") + "m threshold=" + String.format("%.0f", suitableDistanceThreshold) + "m");
                    allSegmentsRouted = false;
                    break;
                }
                
                boolean usedChainedSnap = (wpIdx > 0 && previousLegToSnap != null && bestFromSnap.getClosestNode() == previousLegToSnap.getClosestNode());
                System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] obs " + fromOriginalIdx + " -> " + toOriginalIdx + ": from=" + bestFromSnap.getClosestNode() + (usedChainedSnap ? "(chained)" : "(alt)") + " to=" + bestToSnap.getClosestNode() + " dist=" + String.format("%.0f", bestLegPath.getDistance()) + "m");
                
                // Track boundary nodes
                if (wpIdx == 0) segmentStartNode = bestFromSnap.getClosestNode();
                if (wpIdx == waypoints.size() - 2) segmentEndNode = bestToSnap.getClosestNode();
                
                // Add snaps to routedPathSnaps
                Integer fromFilteredPos = originalToFilteredPos.get(fromOriginalIdx);
                Integer toFilteredPos = originalToFilteredPos.get(toOriginalIdx);
                if (fromFilteredPos != null) routedPathSnaps.get(fromFilteredPos).add(bestFromSnap);
                if (toFilteredPos != null) routedPathSnaps.get(toFilteredPos).add(bestToSnap);
                
                // Handle chaining
                if (wpIdx > 0 && previousLegToSnap != null
                        && bestFromSnap.getClosestNode() != previousLegToSnap.getClosestNode()) {
                    
                    int prevEndNode = previousLegToSnap.getClosestNode();
                    int curStartNode = bestFromSnap.getClosestNode();
                    
                    try {
                        List<Path> intraBridge = router.calcPaths(queryGraph, prevEndNode, EdgeIterator.ANY_EDGE,
                                new int[]{curStartNode}, new int[]{EdgeIterator.ANY_EDGE});
                        if (!intraBridge.isEmpty() && intraBridge.get(0).isFound()
                                && intraBridge.get(0).getDistance() <= MAX_BRIDGE_DISTANCE) {
                            segmentEdges.addAll(intraBridge.get(0).calcEdges());
                            System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] intra-bridge OK: " + prevEndNode + " -> " + curStartNode + " dist=" + String.format("%.0f", intraBridge.get(0).getDistance()) + "m");
                        } else {
                            double bridgeDist = (!intraBridge.isEmpty() && intraBridge.get(0).isFound()) ? intraBridge.get(0).getDistance() : -1;
                            System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] intra-bridge FAIL: " + prevEndNode + " -> " + curStartNode + " dist=" + String.format("%.0f", bridgeDist) + "m (limit=" + MAX_BRIDGE_DISTANCE + ")");
                            segmentSpliceable = false;
                        }
                    } catch (Exception e) {
                        System.out.println("  [Seg " + segNum + " leg " + wpIdx + "] intra-bridge EXCEPTION: " + prevEndNode + " -> " + curStartNode + " err=" + e.getMessage());
                        segmentSpliceable = false;
                    }
                    
                    if (!segmentSpliceable) break;
                }
                
                segmentEdges.addAll(bestLegPath.calcEdges());
                prevChainSnap = previousLegToSnap;
                previousLegToSnap = bestToSnap;
                lastIterEdgeCount = segmentEdges.size() - edgesAtIterStart;
                lastLegRouteDist = bestLegPath.getDistance();
                wpIdx++;
            }
            
            // Salvage partial segment: if the leg loop exited early but committed edges exist,
            // find the last edge arrival in segmentEdges that is already in bestPathNodeSet.
            // Trim the segment there so no walk-forward bridge is needed.
            if (segmentSpliceable && !segmentEdges.isEmpty() && segmentEndNode == -1 && previousLegToSnap != null) {
                int candidateEndNode = previousLegToSnap.getClosestNode();
                if (bestPathNodeSet.contains(candidateEndNode)) {
                    segmentEndNode = candidateEndNode;
                } else {
                    // Scan segment edges for the last arrival that is on the bestPath
                    int prevNode = segmentEdges.get(0).getBaseNode();
                    int foundNode = -1;
                    int trimToEdge = -1;
                    for (int e = 0; e < segmentEdges.size(); e++) {
                        EdgeIteratorState edge = segmentEdges.get(e);
                        int arrival = (edge.getBaseNode() == prevNode) ? edge.getAdjNode() : edge.getBaseNode();
                        if (bestPathNodeSet.contains(arrival)) {
                            foundNode = arrival;
                            trimToEdge = e;
                        }
                        prevNode = arrival;
                    }
                    if (foundNode >= 0) {
                        while (segmentEdges.size() > trimToEdge + 1) segmentEdges.remove(segmentEdges.size() - 1);
                        segmentEndNode = foundNode;
                        System.out.println("  Segment end trimmed: last bestPath node in segment=" + segmentEndNode + " (edge " + trimToEdge + ")");
                    } else {
                        // Fall back: seed from chain end and let walk-forward splice attempt
                        segmentEndNode = candidateEndNode;
                        // Diagnostics: log the last 5 arrival nodes to understand why none matched bestPathNodeSet
                        StringBuilder dbg = new StringBuilder("  Segment end candidate=" + segmentEndNode + " (not in bestPath). Last 5 arrivals: ");
                        int prevNode2 = segmentEdges.get(0).getBaseNode();
                        List<Integer> arrivals = new ArrayList<>();
                        for (EdgeIteratorState edge : segmentEdges) {
                            int arr = (edge.getBaseNode() == prevNode2) ? edge.getAdjNode() : edge.getBaseNode();
                            arrivals.add(arr);
                            prevNode2 = arr;
                        }
                        for (int i = Math.max(0, arrivals.size() - 5); i < arrivals.size(); i++) {
                            dbg.append(arrivals.get(i)).append("(bp=").append(bestPathNodeSet.contains(arrivals.get(i))).append(") ");
                        }
                        System.out.println(dbg);
                    }
                }
            }

            // If segment has internal gaps (e.g. failed intra-bridge), mark unspliceable
            if (!segmentSpliceable) {
                segmentStartNode = -1;
                segmentEndNode = -1;
            }
            
            // Walk-back splice for start and end nodes if segment is still spliceable
            if (segmentSpliceable) {
                List<Integer> segWaypoints = segmentWaypointIndices.get(segNum);
                double maxBridgeDistance = 20000; // 20km max bridge
                
                // Start anchor walk-back
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
                        System.out.println("  WARNING: could not find walk-back splice for start of segment " + segNum);
                        segmentStartNode = -1;
                    }
                }
                
                // End anchor walk-forward
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
                        System.out.println("  WARNING: could not find walk-forward splice for end of segment " + segNum);
                        segmentEndNode = -1;
                    }
                }
            }
            
            perSegmentRoutedEdges.add(segmentEdges);
            segmentBoundaryNodes.add(new int[]{segmentStartNode, segmentEndNode});
            
            // Print routed segment as GeoJSON for debugging
            if (!segmentEdges.isEmpty()) {
                StringBuilder segmentGeoJson = new StringBuilder();
                segmentGeoJson.append("{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
                boolean first = true;
                for (EdgeIteratorState edge : segmentEdges) {
                    PointList edgePoints = edge.fetchWayGeometry(FetchMode.ALL);
                    for (int i = 0; i < edgePoints.size(); i++) {
                        if (!first) segmentGeoJson.append(",");
                        segmentGeoJson.append("[").append(edgePoints.getLon(i)).append(",").append(edgePoints.getLat(i)).append("]");
                        first = false;
                    }
                }
                double segmentDistance = 0;
                for (EdgeIteratorState edge : segmentEdges) {
                    segmentDistance += edge.getDistance();
                }
                segmentGeoJson.append("]},\"properties\":{\"stroke\":\"#ff9900\",\"path_type\":\"via_waypoint_segment\",\"segment_index\":")
                        .append(segNum)
                        .append(",\"edges\":")
                        .append(segmentEdges.size())
                        .append(",\"distance\":")
                        .append(segmentDistance)
                        .append(",\"waypoints\":")
                        .append(waypoints.size())
                        .append(",\"spliceable\":")
                        .append(segmentSpliceable)
                        .append(",\"start_node\":")
                        .append(segmentStartNode)
                        .append(",\"end_node\":")
                        .append(segmentEndNode)
                        .append("}}");
                System.out.println("Via-waypoint Segment #" + segNum + " GeoJSON: " + segmentGeoJson);
            }
            
            // If splice attempts left either boundary invalid, prevent Viterbi fallback:
            // the segment is simply unspliceable and buildMergedEdgeList will use bestPath instead
            if (segmentStartNode == -1 || segmentEndNode == -1) {
                allSegmentsRouted = true;
            }

            if (!allSegmentsRouted) break;
        }
        
        if (!allSegmentsRouted || perSegmentRoutedEdges.isEmpty()) {
            return new ViaWaypointRoutingResult(false, null, null, null);
        }
        
        return new ViaWaypointRoutingResult(true, perSegmentRoutedEdges, segmentBoundaryNodes, routedPathSnaps);
    }

    @NotNull
    private static List<List<Integer>> buildWaypointsList(List<Observation> filteredObservations, List<List<Integer>> offPathSegments, Set<Integer> offPathSet) {
        List<List<Integer>> segmentWaypointIndices = new ArrayList<>();
        
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
            
            List<Integer> waypoints = new ArrayList<>();

            // Add anchor before
            for (int i = seg.get(0) - 1; i >= 0; i--) {
                if (!offPathSet.contains(i)) {
                    waypoints.add(filteredObservations.get(i).getPoint().index);
                    break;
                }
            }

            // Add off-path observations
            for (int offPathIdx : seg) {
                waypoints.add(filteredObservations.get(offPathIdx).getPoint().index);
            }

            // Add anchor after
            for (int i = seg.get(seg.size() - 1) + 1; i < filteredObservations.size(); i++) {
                if (!offPathSet.contains(i)) {
                    waypoints.add(filteredObservations.get(i).getPoint().index);
                    break;
                }
            }

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
}