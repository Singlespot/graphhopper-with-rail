package fr.singlespot.railway_matching;

import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.matching.Observation;
import com.graphhopper.storage.index.Snap;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class SnappedPointsAnalyzer {
    public static class AnalysisResult {
        public final int[] snapsOnPathCounts;
        public final List<List<Boolean>> snapsNotOnRoutedPaths;
        public final List<Set<Integer>> routedPathsPathEdgeIndices;
        public final List<List<List<Snap>>> snapsPerObservationOnRoutedPathTmpList;

        public AnalysisResult(int[] snapsOnPathCounts,
                              List<List<Boolean>> snapsNotOnRoutedPaths,
                              List<Set<Integer>> routedPathsPathEdgeIndices,
                              List<List<List<Snap>>> snapsPerObservationOnRoutedPathTmpList) {
            this.snapsOnPathCounts = snapsOnPathCounts;
            this.snapsNotOnRoutedPaths = snapsNotOnRoutedPaths;
            this.routedPathsPathEdgeIndices = routedPathsPathEdgeIndices;
            this.snapsPerObservationOnRoutedPathTmpList = snapsPerObservationOnRoutedPathTmpList;
        }
    }

    /**
     * Analyze how many snaps are on each routed path.
     * @param routedPaths List of candidate paths
     * @param filteredObservations List of filtered observations
     * @param findCandidateSnaps Function to find snaps for an observation
     * @param resolveToRealEdge Function to resolve to the real edge (if needed)
     * @return AnalysisResult with counts and mappings
     */
    public AnalysisResult analyze(List<Path> routedPaths,
                                  List<Observation> filteredObservations,
                                  java.util.function.Function<Observation, List<Snap>> findCandidateSnaps,
                                  java.util.function.Function<EdgeIteratorState, EdgeIteratorState> resolveToRealEdge) {
        List<List<Snap>> snapsPerObservationTmp = filteredObservations.stream()
                .map(findCandidateSnaps)
                .collect(Collectors.toList());
        List<List<Boolean>> snapsNotOnRoutedPaths = IntStream.range(0, snapsPerObservationTmp.size())
                .mapToObj(i -> IntStream.range(0, routedPaths.size()).mapToObj(j -> false).collect(Collectors.toList()))
                .collect(Collectors.toList());
        List<Set<Integer>> routedPathsPathEdgeIndices = IntStream.range(0, routedPaths.size()).mapToObj(i -> new HashSet<Integer>()).collect(Collectors.toList());
        int[] snapsOnPathCounts = new int[routedPaths.size()];
        List<List<List<Snap>>> snapsPerObservationOnRoutedPathTmpList = new ArrayList<>();

        for (int routedPathsIndex = 0, routedPathsSize = routedPaths.size(); routedPathsIndex < routedPathsSize; routedPathsIndex++) {
            Path tmpRoutedPath = routedPaths.get(routedPathsIndex);
            List<List<Snap>> snapsPerObservationOnRoutedPathTmp = new ArrayList<>();
            Set<Integer> pathEdgeIndices = routedPathsPathEdgeIndices.get(routedPathsIndex);
            if (!tmpRoutedPath.isFound()) {
                snapsPerObservationOnRoutedPathTmpList.add(snapsPerObservationOnRoutedPathTmp);
                continue;
            }
            List<EdgeIteratorState> pathEdges = tmpRoutedPath.calcEdges();
            int maxEdgeIndex = -1;
            int snapsOnPathCount = 0;
            for (int snapsIndex = 0, snapsPerObservationTmpSize = snapsPerObservationTmp.size(); snapsIndex < snapsPerObservationTmpSize; snapsIndex++) {
                List<Snap> snaps = snapsPerObservationTmp.get(snapsIndex);
                boolean oneOfSnapsOnRoutedPath = false;
                for (Snap snap : snaps) {
                    if (oneOfSnapsOnRoutedPath) break;
                    for (int edgeIndex = 0; edgeIndex < pathEdges.size(); edgeIndex++) {
                        EdgeIteratorState e = pathEdges.get(edgeIndex);
                        EdgeIteratorState pathEdge = resolveToRealEdge.apply(e);
                        EdgeIteratorState snapEdge = snap.getClosestEdge();
                        if (pathEdge.getEdge() == snapEdge.getEdge()) {
                            if (edgeIndex > maxEdgeIndex || snapsIndex == snapsPerObservationTmpSize - 1) {
                                snapsPerObservationOnRoutedPathTmp.add(Collections.singletonList(snap));
                                maxEdgeIndex = edgeIndex;
                                pathEdgeIndices.add(edgeIndex);
                            }
                            oneOfSnapsOnRoutedPath = true;
                            snapsOnPathCount++;
                            break;
                        }
                    }
                }
                if (!oneOfSnapsOnRoutedPath) {
                    snapsNotOnRoutedPaths.get(snapsIndex).set(routedPathsIndex, true);
                }
            }
            snapsOnPathCounts[routedPathsIndex] = snapsOnPathCount;
            snapsPerObservationOnRoutedPathTmpList.add(snapsPerObservationOnRoutedPathTmp);
        }
        return new AnalysisResult(snapsOnPathCounts, snapsNotOnRoutedPaths, routedPathsPathEdgeIndices, snapsPerObservationOnRoutedPathTmpList);
    }
}
