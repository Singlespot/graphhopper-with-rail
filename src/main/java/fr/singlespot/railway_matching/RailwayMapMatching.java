package fr.singlespot.railway_matching;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.*;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.graphhopper.util.StopWatch;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
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
     * @param routed_path  The routed path between the first and last observation
     */
    public MatchResult match_with_routing(List<Observation> observations, StopWatch sw, Path routed_path) {
        return match_with_routing(observations, false, 0, sw, routed_path);
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
     * @param routed_path  The routed path between the first and last observation
     */
    public MatchResult match_with_routing(List<Observation> observations, boolean ignoreErrors, int offset, StopWatch sw,
                                          Path routed_path) {
        this.offset = offset;
        resetCounters(observations.size(), offset);
        List<Observation> observationSubList = observations.subList(offset, observations.size());
        List<Observation> filteredObservations = filterObservations(observationSubList);
        statistics.put("filteredObservations", filteredObservations.size());

        // Snap observations to links. Generates multiple candidate snaps per observation.
        List<List<Snap>> snapsPerObservation = filteredObservations.stream()
                .map(o -> findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, o.getPoint().accuracy))
                .collect(Collectors.toList());
        statistics.put("snapsPerObservation", snapsPerObservation.stream().mapToInt(Collection::size).toArray());

        // Create the query graph, containing split edges so that all the places where an observation might have happened
        // are a node. This modifies the Snap objects and puts the new node numbers into them.
        queryGraph = QueryGraph.create(graph, snapsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));

        MatchResult result;
        // check if at least one snap is on routed_path
        if (routed_path != null) {
            List<EdgeIteratorState> edgeIteratorStates = routed_path.calcEdges();
            boolean anySnapNotOnRoutedPath = snapsPerObservation.stream().flatMap(Collection::stream).anyMatch(s -> edgeIteratorStates.stream().noneMatch(e -> e.getEdge() == s.getClosestEdge().getEdge()
                    || (e instanceof VirtualEdgeIteratorState && graph.getEdgeIteratorStateForKey(((VirtualEdgeIteratorState) e).getOriginalEdgeKey()).getEdge() == s.getClosestEdge().getEdge())));
//            TODO
            if (false) {
                int snapsIndex = 0;
                for (List<Snap> snaps : snapsPerObservation) {
                    //  if no edgeIteratorState.getEdge() == snap.getClosestEdge().getEdge()
                    if (edgeIteratorStates.stream().noneMatch(e -> snaps.stream().anyMatch(s -> e.getEdge() == s.getClosestEdge().getEdge()
                            || (e instanceof VirtualEdgeIteratorState && graph.getEdgeIteratorStateForKey(((VirtualEdgeIteratorState) e).getOriginalEdgeKey()).getEdge() == s.getClosestEdge().getEdge())))) {
                        System.out.println("observation not on routed path: " + snaps.get(0).getQueryPoint());
//                            break;
                    }
                    snapsIndex++;
                }

                // Creates candidates from the Snaps of all observations (a candidate is basically a
                // Snap + direction).
                List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, snapsPerObservation);
                // Compute the most likely sequence of map matching candidates:
                List<SequenceState<State, Observation, Path>> seq = computeViterbiSequence(timeSteps, ignoreErrors, sw);
                statistics.put("transitionDistances", seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> Math.round(s.transitionDescriptor.getDistance())).toArray());
                statistics.put("visitedNodes", router.getVisitedNodes());
                statistics.put("snapDistanceRanks", IntStream.range(0, seq.size()).map(i -> snapsPerObservation.get(i).indexOf(seq.get(i).state.getSnap())).toArray());
                statistics.put("snapDistances", seq.stream().mapToDouble(s -> s.state.getSnap().getQueryDistance()).toArray());
                statistics.put("maxSnapDistances", IntStream.range(0, seq.size()).mapToDouble(i -> snapsPerObservation.get(i).stream().mapToDouble(Snap::getQueryDistance).max().orElse(-1.0)).toArray());

                List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());

                result = new MatchResult(prepareEdgeMatches(seq));
                Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
                result.setMergedPath(new MapMatchedPath(queryGraph, queryGraphWeighting, path));
                result.setMatchMillis(seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> s.transitionDescriptor.getTime()).sum());
                result.setMatchLength(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum());
                result.setGPXEntriesLength(gpxLength(observations));
                result.setGraph(queryGraph);
                result.setWeighting(queryGraphWeighting);
            } else {
                result = new MatchResult(new ArrayList<EdgeMatch>());
                result.setMatchMillis(routed_path.getTime());
                result.setMatchLength(routed_path.getDistance());
                result.setGPXEntriesLength(gpxLength(observations));
                result.setGraph(queryGraph);
                result.setWeighting(queryGraph.wrapWeighting(router.getWeighting()));
                result.setMergedPath(routed_path);
                processedUpTo=observations.size();
            }
            return result;
        }
//        TODO
        return null;
    }
}