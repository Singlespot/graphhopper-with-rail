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
    public MatchResult match_with_routing(List<Observation> observations, StopWatch sw, List<Path> routedPaths) {
        return match_with_routing(observations, false, 0, sw, routedPaths);
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
                                          List<Path> routedPaths) {
        this.offset = offset;
        resetCounters(observations.size(), offset);
        List<Observation> observationSubList = observations.subList(offset, observations.size());
        List<Observation> filteredObservations = filterObservations(observationSubList);
        statistics.put("filteredObservations", filteredObservations.size());

        // Snap observations to links. Generates multiple candidate snaps per observation.
        List<List<Snap>> snapsPerObservationTmp = filteredObservations.stream()
                .map(o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, o.getPoint().accuracy))
                .collect(Collectors.toList());
        queryGraph = QueryGraph.create(graph, snapsPerObservationTmp.stream().flatMap(Collection::stream).collect(Collectors.toList()));

        MatchResult result;
        List<SequenceState<State, Observation, Path>> seq;
        Path routedPath = null;
        boolean anySnapNotOnAnyRoutedPath = false;
        List<List<Snap>> snapsPerObservationOnRoutedPath = new ArrayList<>();

        if (routedPaths.get(0) != null && routedPaths.stream().anyMatch(Path::isFound)) {
            List<List<Boolean>> snapsNotOnRoutedPaths = IntStream.range(0, snapsPerObservationTmp.size())
                    .mapToObj(i -> IntStream.range(0, routedPaths.size()).mapToObj(j -> false).collect(Collectors.toList()))
                    .collect(Collectors.toList());
            List<Set<Integer>> routedPathsPathEdgeIndices = IntStream.range(0, routedPaths.size()).mapToObj(i -> new HashSet<Integer>()).collect(Collectors.toList());

            // Loop over all routed paths to find the routed path that matches all observations
            for (int routedPathsIndex = 0, routedPathsSize = routedPaths.size(); routedPathsIndex < routedPathsSize; routedPathsIndex++) {
                // check if all snaps are on routed_path
                Path tmpRoutedPath = routedPaths.get(routedPathsIndex);
                List<List<Snap>> snapsPerObservationOnRoutedPathTmp = new ArrayList<>();
                Set<Integer> pathEdgeIndices = routedPathsPathEdgeIndices.get(routedPathsIndex);
                if (!tmpRoutedPath.isFound()) {
                    continue;
                }
                List<EdgeIteratorState> pathEdges = tmpRoutedPath.calcEdges();
//                IntIndexedContainer pathNodes = routedPath.calcNodes();
//                PointList pathPoints = routedPath.calcPoints();
                int maxEdgeIndex = -1;

                // find the snap for each observation that is on the routed_path
                for (int snapsIndex = 0, snapsPerObservationTmpSize = snapsPerObservationTmp.size(); snapsIndex < snapsPerObservationTmpSize; snapsIndex++) {
                    List<Snap> snaps = snapsPerObservationTmp.get(snapsIndex);
                    boolean oneOfSnapsOnRoutedPath = false;
                    for (Snap snap : snaps) {
                        if (oneOfSnapsOnRoutedPath) break;
                        for (int edgeIndex = 0; edgeIndex < pathEdges.size(); edgeIndex++) {
                            EdgeIteratorState e = pathEdges.get(edgeIndex);
                            EdgeIteratorState pathEdge = resolveToRealEdge(e);
                            EdgeIteratorState snapEdge = snap.getClosestEdge();
                                  if (pathEdge.getEdge() == snapEdge.getEdge()) {
                                //  Add a list with just snap since we know it is the snap on the path
                                if (edgeIndex > maxEdgeIndex || snapsIndex == snapsPerObservationTmpSize - 1) {
                                    snapsPerObservationOnRoutedPathTmp.add(Collections.singletonList(snap));
                                    maxEdgeIndex = edgeIndex;
                                    pathEdgeIndices.add(edgeIndex);
                                }
                                oneOfSnapsOnRoutedPath = true;
                                break;
                            }
                        }
                    }
                    if (!oneOfSnapsOnRoutedPath) {
                        snapsNotOnRoutedPaths.get(snapsIndex).set(routedPathsIndex, true);
                    }
                }

                if (pathEdgeIndices.size() <= 2)
                    System.out.println("Path # " + (routedPathsIndex + 1) + ", all snaps on the first and last edges");
                int finalRoutedPathsIndex = routedPathsIndex;
                boolean allSnapsOnRoutedPath = snapsNotOnRoutedPaths.stream().noneMatch(snap -> snap.get(finalRoutedPathsIndex));
                // We make sure that all observations are on the same path, and they are not all snapped on first and last segments unless there is only 2 observations
                if (allSnapsOnRoutedPath && (pathEdgeIndices.size() > 2 || filteredObservations.size() == 2)) {
                    routedPath = tmpRoutedPath;
                    snapsPerObservationOnRoutedPath.addAll(snapsPerObservationOnRoutedPathTmp);
                    System.out.println("All observations on the path #" + (finalRoutedPathsIndex + 1) + ": using direct routing for map matching");
                    break;
                }
            }

            for (int observationsIndex = 0; observationsIndex < filteredObservations.size(); observationsIndex++) {
                List<Boolean> snapNotOnRoutedPath = snapsNotOnRoutedPaths.get(observationsIndex);
                if (snapNotOnRoutedPath.stream().allMatch(Boolean::booleanValue)) {
                    System.out.println("Observation not on any path: " + snapsPerObservationTmp.get(observationsIndex).get(0).getQueryPoint());
                    anySnapNotOnAnyRoutedPath = true;
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