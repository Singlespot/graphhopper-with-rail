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
        java.util.function.Function<Observation, List<Snap>> findCandidateSnaps = o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, o.getPoint().accuracy);
        // Will generate snapsPerObservationTmp inside analyzer
        queryGraph = null; // Will set after analysis

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

            // Now set queryGraph using the generated snapsPerObservationTmp
            List<List<Snap>> snapsPerObservationTmp = filteredObservations.stream().map(findCandidateSnaps).collect(Collectors.toList());
            queryGraph = QueryGraph.create(graph, snapsPerObservationTmp.stream().flatMap(Collection::stream).collect(Collectors.toList()));

            // Variables to track the path with the most snaps (for forceInitialRouting)
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

                // If we're forcing routing, track the path with the most snaps
                if (forceInitialRouting && snapsOnPathCount > maxSnapsCount &&
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
            for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
                List<Boolean> snapNotOnRoutedPath = snapsNotOnRoutedPaths.get(observationsIndex);
                List<Snap> snaps = snapsPerObservationTmp.get(observationsIndex);
                if (snapNotOnRoutedPath.stream().allMatch(Boolean::booleanValue) && !snaps.isEmpty()) {
                    System.out.println("Observation not on any path: " + snapsPerObservationTmp.get(observationsIndex).get(0).getQueryPoint());
                    if (!forceInitialRouting) {
                        anySnapNotOnAnyRoutedPath = true;
                    }
                }
            }
        }
        if (anySnapNotOnAnyRoutedPath || routedPath == null) {
            // Creates candidates from the Snaps of all observations (a candidate is basically a
            // Snap + direction). We need to put lower the accuracy to a max value of 300
            List<List<Snap>> snapsPerObservation = filteredObservations.stream()
                    .map(o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, Math.min(o.getPoint().accuracy, 300.0)))
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