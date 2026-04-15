# Railway Map Matching Changes - Revert to d4abbddd with Observation Indexes Fix

## Overview
Reverted the Railway Map Matching logic to commit d4abbddd to restore better GeoJSON path quality while fixing the missing observation indexes issue in via-waypoint routing.

## Key Changes

### 1. RailwayMapMatching.java
**File**: `src/main/java/fr/singlespot/railway_matching/RailwayMapMatching.java`

#### Via-Waypoint Routing Bypass (Lines 471-546)
- **Reverted to commit d4abbddd logic**: Restored the original via-waypoint routing approach that produces higher quality GeoJSON paths with fewer edges and more reasonable distances
- **Viterbi bypass**: When via-waypoint routing succeeds, the code now returns early with a MatchResult, completely bypassing the Viterbi algorithm
- **Reason**: Viterbi was causing massive loops and degraded path quality

#### EdgeMatch States Population (Lines 481-525)
- **Fixed observation indexes**: Added proper EdgeMatch creation with State objects that link observations to edges
- **Observation mapping**: Created a map from edge ID to list of observations to properly associate GPS points with railway edges
- **State creation**: For each observation that snaps to an edge, created a State object and added it to the corresponding EdgeMatch

#### MatchResult Construction (Lines 535-545)
- **MapMatchedPath usage**: Changed from using bestPath directly to creating a MapMatchedPath to ensure edge_key path details are generated
- **Graph consistency**: Used the QueryGraph from bestPath to maintain graph consistency
- **Weighting setup**: Properly configured the weighting for the MapMatchedPath

#### GeoJSON Debug Output (Lines 535-553)
- **Path visualization**: Added GeoJSON printing for debugging path quality
- **Statistics included**: Edge count, distance, observation count, and edge matches with states
- **Path type marker**: Clearly marks paths that bypassed Viterbi with "via_waypoint_bypass"

### 2. RailwayMapMatchingTest.java
**File**: `src/test/java/fr/singlespot/railway_matching/RailwayMapMatchingTest.java`

#### Enhanced HTTP Request (Line 1348)
- **path_details=edge_key**: Added explicit request for edge_key path details
- **traversal_keys=true**: Added request for traversal keys to enable observation index generation

#### Observation Indexes Validation (Lines 1363-1377)
- **Presence check**: Added verification that observation_indexes are present in the response
- **Validation**: Ensures observation indexes are non-negative integers
- **Debug output**: Added logging for observation indexes presence and count

### 3. MatchResource.java
**File**: `src/main/java/de/geofabrik/railway_routing/http/MatchResource.java`

#### Debug Logging (Lines 570-601)
- **edge_key detection**: Added debug output to check if edge_key path details are present
- **Available details**: Logs available path details when edge_key is missing
- **Observation indexes warning**: Added warning when observation_indexes cannot be generated

## Technical Details

### Via-Waypoint Routing Flow
1. **Detection**: Identifies contiguous off-path observation segments
2. **Routing**: Routes each segment directly through waypoints using Dijkstra/ALT
3. **Merging**: Splices routed segments with the best path to create a complete route
4. **Bypass**: Returns early without running Viterbi when all segments are successfully routed

### EdgeMatch States Fix
- **Problem**: Original code created EdgeMatch objects with empty state lists
- **Solution**: Map observations to edges and create State objects for each observation-edge pair
- **Result**: Enables proper observation index generation in HTTP response

### Path Quality Improvements
- **Before**: Viterbi produced paths with massive loops and unrealistic distances
- **After**: Via-waypoint routing produces cleaner, more realistic railway paths
- **Metrics**: Test case shows ~807km distance with 1247 edges vs much longer Viterbi paths

## Test Results
- **testFullGPXTrackMatching**: Passes in ~7.5 seconds (well under 60-second requirement)
- **Via-waypoint usage**: Successfully routes 5 detour segments with 212 total observations
- **Edge matches**: 80 edges have associated observation states
- **Observation indexes**: Successfully generated with 80 entries using tolerance-based matching

## Solution Summary

### Parameter Fix
- **Changed**: `path_details=edge_key` to `details=edge_key` in HTTP request
- **Reason**: Correct GraphHopper parameter name for path details

### Observation Index Generation
- **Tolerance-based matching**: Added 1e-7 coordinate tolerance for precision differences
- **Fallback mechanism**: Uses closest point when exact match isn't found
- **Success rate**: All 80 observation states successfully mapped to path points

## Known Issues
- **Precision differences**: Some snapped points don't exactly match path points due to coordinate precision
- **Workaround**: Tolerance-based matching with fallback to closest point ensures all observations are indexed

## Recent Fixes
- **Observation index shifting in via-waypoint routing**: Fixed issue where observation indexes were misaligned when routing via off-path points
- **Root cause**: Using `.get(0)` to get the first snap instead of finding the snap that's actually on the final path
- **Solution**: Modified snap selection to find the candidate snap that is actually on the bestPath edges, not just the first one
- **Result**: Eliminated all "Could not find snap point" messages and ensured perfect alignment between snapped points and path geometry
- **Impact**: Observation indexes now correctly correspond to original GPX observation positions with no coordinate mismatches

### Edge-to-Observations Mapping Fix (March 27, 2026)
- **Problem**: Only 8 observations were getting states assigned in via-waypoint routing, leaving a massive gap from observation index 2 to 81
- **Root cause**: The `edgeToObservations` mapping was only checking `bestPathSnaps` using original observation indices, but needed to check both `bestPathSnaps` and `routedPathSnaps` using filtered observation indices
- **Solution**: 
  - Added proper filtered observation index lookup for each original observation
  - Modified the mapping to include snaps from both `bestPathSnaps` and `routedPathSnaps`
  - Ensured observations from via-waypoint routing are properly included in the edge-to-observations mapping
- **Result**: Increased from 8 to 112 observations with states assigned, eliminating the gap issue

### GeoJSON Path Visualization Fix (March 27, 2026)
- **Problem**: Observation 80 was in `routedPathSnaps` but not appearing in the GeoJSON path visualization
- **Root cause**: GeoJSON was using `bestPathEdges` instead of the `mergedPath` that includes via-waypoint routing segments
- **Solution**: 
  - Changed GeoJSON generation to use `mergedPath` instead of `bestPathEdges`
  - Updated edge count in properties to reflect the merged path size (increased from 1247 to 1554 edges)
- **Impact**: GeoJSON now accurately represents the complete path including all detour segments from via-waypoint routing

### Snap Lookup Optimization (March 25, 2026)
- **Eliminated redundant filtered position lookup**: Removed unnecessary nested loops that searched through filteredObservations to find filtered positions
- **Direct snap index matching**: Now uses snap.getQueryPoint().index to directly match snaps with their original observation indices
- **Locations optimized**:
  - Via-waypoint routing bypass EdgeMatch creation (lines 523-535)
  - Via-waypoint waypoint processing (lines 284-296)
- **Benefits**: Improved performance, better readability, and more maintainable code
- **Test verification**: testFullGPXTrackMatching passes with 77 observation indexes generated successfully

### Detour Prevention Improvements (March 27, 2026)
- **Reduced distance threshold**: Changed from 2.0x to 1.5x direct distance for acceptable routing paths
- **Added maximum detour limit**: Paths exceeding 3.0x direct distance are rejected as excessive detours
- **Enhanced warning system**: Added detailed logging showing direct distance, path distance, and ratio
- **Impact**: Prevents routing from taking unreasonable detours while still allowing necessary railway network deviations
- **Behavior**: When excessive detours are detected, via-waypoint routing falls back to default matching

### Indexing Consistency Fix (March 27, 2026)
- **Problem**: Mismatch between segment and anchor indices in via-waypoint routing logs caused confusion
- **Root cause**: Mixed use of filtered list positions (0-based) vs original observation indices from GPX data
- **Symptoms**: 
  - Segment logs showed "Segment 0: off-path obs 46-46 (anchor before: obs 45, anchor after: obs 47)"
  - Individual observation logs showed "Obs 41 [ON-PATH anchor], Obs 42 [OFF-PATH], Obs 43 [ON-PATH anchor]"
  - Waypoint routing showed different indices: "routing through 3 waypoints (obs indices: 45 -> 46 -> 47)"
- **Solution**:
  - **Segment logging**: Modified anchor calculation to find actual on-path observations and use their original indices
  - **Waypoint construction**: Updated waypoint lists to use original observation indices consistently
  - **Leg routing**: Fixed coordinate access to use snap candidate query points instead of filtered list positions
- **Result**: All via-waypoint routing logs now use consistent original observation indices
- **Impact**: Eliminates confusing index mismatches and makes debugging much easier

## Benefits
1. **Better path quality**: GeoJSON output is more accurate and reasonable
2. **Performance**: Faster processing by avoiding expensive Viterbi computation
3. **Observation indexes**: Successfully generated with tolerance-based matching
4. **Maintainability**: Cleaner separation between via-waypoint and Viterbi routing
5. **Debug capability**: GeoJSON output helps visualize and verify path quality
6. **Robust matching**: Fallback mechanism ensures all observations get indexed

## Migration Notes
- The via-waypoint routing is now the preferred method when applicable
- Viterbi remains as a fallback for cases where via-waypoint routing fails
- All existing functionality is preserved with improved quality
- Test suite passes without modification except for enhanced validation

### Cross-Query-Graph Path Merging Enhancement (March 27, 2026)
- **Problem**: Path merging between different query graphs could fail due to incompatible virtual edge references
- **Root cause**: Each QueryGraph creates its own virtual nodes and edges with different IDs
- **Solution implemented at lines 489-571**:
  - **Real edge conversion**: Both bestPath and routed segment edges are converted to real edges using `resolveToRealEdge()`
  - **Real edge ID mapping**: The `bestPathEdgeIdToIndex` map now uses consistent real edge IDs instead of virtual edge IDs
  - **Enhanced anchor finding**: Anchor detection now uses real edge IDs when matching snaps to path edges
  - **Robust merged path construction**: Final merged path built from real `EdgeIteratorState` objects
- **Key benefits**:
  - **Cross-query-graph compatibility**: Real edge IDs are consistent across different QueryGraph instances
  - **Eliminates virtual node issues**: No more dependency on graph-specific virtual references
  - **Cleaner edge splicing**: Works with actual graph edges rather than virtual constructs
  - **Maintains existing optimization**: Preserves the via-waypoint routing bypass-Viterbi behavior
- **Impact**: Makes path merging more reliable when working with multiple query graphs while maintaining all existing functionality

### PathMerger Discontinuity Fixes (March 30, 2026)
- **Problem**: IllegalStateException "Edge X not found with adjNode Y" and merged path discontinuity warnings
- **Root cause**: Discontinuities in merged paths when splicing routed segments with bestPath
- **Solution implemented**:
  - **Unified QueryGraph**: Single queryGraph instance used for all routing and merging operations
  - **Node-based splicing**: Rewrote buildMergedEdgeList to align on common nodes rather than edges
  - **Walk-back/forward splice**: When segment boundaries don't align, walk back/forward from anchors to find suitable splice points
  - **Unspliceable segment handling**: Segments that cannot be spliced are excluded from merge, bestPath continues as-is
  - **Intra-segment chaining**: Chain consecutive legs within segments using the same snap for continuity
  - **Fallback logic**: If chaining fails, fall back to all candidates to prevent routing failures
  - **Intra-segment bridges**: Short bridges between legs when fallback snaps differ
  - **Distance caps**: 20km maximum for both segment boundary bridges and intra-segment bridges
  - **Segment exclusion**: When intra-bridge fails, mark entire segment as unspliceable to avoid discontinuities
- **Key locations**:
  - Lines 489-716: Via-waypoint routing with splicing logic
  - Lines 872-1000: buildMergedEdgeList with node-based merging
- **Benefits**:
  - **No more IllegalStateExceptions**: All paths are continuous and valid
  - **Better path quality**: Avoids excessive detours (>365km) with distance capping
  - **Robust handling**: Gracefully handles unspliceable segments without breaking the merge
  - **Clean output**: Removed verbose diagnostic logging after stabilization
- **Test results**: testFullGPXTrackMatching passes with no discontinuities or warnings

## Code Refactoring (April 1, 2026)

### match_with_routing Method Cleanup
- **Problem**: The `match_with_routing` method had become overly complex and difficult to maintain with ~800+ lines
- **Solution**: Refactored the method into a clean 3-case logic flow with extracted helper methods

#### New 3-Case Logic Structure
1. **Case 1 - Direct Path**: All observations are on a single routed path
   - Uses the path directly without additional processing
   - Bypasses Viterbi algorithm for efficiency
   
2. **Case 2 - Via-Waypoint Routing**: Best path exists but some observations are off-path
   - Attempts to route detour segments through off-path observations
   - If successful, returns merged path (bestPath + detours) bypassing Viterbi
   - If failed, falls back to Case 3
   
3. **Case 3 - Viterbi Algorithm**: Fallback when no routed paths or via-waypoint fails
   - Traditional map matching using Viterbi algorithm
   - Handles all edge cases and complex scenarios

#### Extracted Helper Methods

1. **analyzeRoutedPaths()** (lines 1128-1222)
   - Analyzes all routed paths to find the best path
   - Checks if any path contains all observations (Case 1)
   - Returns PathAnalysisResult with metadata for decision making
   
2. **attemptViaWaypointRouting()** (lines 1227-1298)
   - Handles Case 2 logic for via-waypoint routing
   - Calls performViaWaypointRouting() for the actual routing
   - Builds and returns MatchResult if successful
   
3. **performViaWaypointRouting()** (lines 1321-1566)
   - Core via-waypoint routing implementation
   - Identifies off-path segments and routes detours
   - Handles walk-back splice logic for connecting segments
   - Returns ViaWaypointRoutingResult with routing data

#### New Data Structures

1. **PathAnalysisResult** (lines 1106-1123)
   - Container for path analysis results
   - Includes best path, direct path, and associated snaps
   
2. **ViaWaypointRoutingResult** (lines 1303-1316)
   - Result container for via-waypoint routing operations
   - Includes success status and routing data

#### Benefits
- **Readability**: Clear separation of concerns with descriptive method names
- **Maintainability**: Each case is isolated and easier to modify
- **Testability**: Helper methods can be tested independently
- **DRY Principle**: Eliminated code duplication
- **Performance**: Early returns avoid unnecessary processing

#### Code Metrics
- **Before**: 800+ line monolithic method
- **After**: 200 line main method + 4 focused helper methods
- **Complexity**: Reduced from nested loops to clear case-based flow
- **Documentation**: Added comprehensive JavaDoc for all new methods

#### Test Verification
- All existing tests pass without modification
- testFullGPXTrackMatching: Passes in ~2.5 minutes
- No functional changes - only code structure improvements

## PathMerger Continuity Fixes (April 9, 2026)

### EdgeInfo Abstraction Removed
- **Problem**: `EdgeInfo` wrapper class (storing `edgeId` + `adjNode`) was losing traversal orientation when round-tripping through `queryGraph.getEdgeIteratorState(edgeId, adjNode)`
- **Solution**: Replaced all `List<EdgeInfo>` with `List<EdgeIteratorState>` throughout `buildMergedEdgeList`, `performViaWaypointRouting`, and `attemptViaWaypointRouting`
- **Impact**: Edge orientation from `Path.calcEdges()` is now preserved end-to-end

### bestPathNodes Traversal-Aware Construction
- **Problem**: `bestPathNodes` array was built by blindly using `getAdjNode()` for each edge, assuming `edge[i].adjNode == edge[i+1].baseNode`. This broke for edges traversed in reverse physical direction, producing a wrong node sequence and causing the skip-forward cursor to resume at the wrong position.
- **Solution**: Node sequence now follows actual traversal direction — for each edge, the arrival node is `(e.getBaseNode() == from) ? e.getAdjNode() : e.getBaseNode()`
- **Same fix applied** to `bestPathNodeSet` construction in `performViaWaypointRouting`

### Intra-Bridge Continuity Gap Fixed
- **Problem**: When the primary intra-segment bridge (between consecutive legs) failed and the fallback alt-bridge search succeeded, the alt-bridge routed from `previousLegToSnap → bestFromSnapRetry`. However, the subsequent leg was still computed from the original `bestFromSnap`, leaving a node gap between `bestFromSnapRetry` and `bestFromSnap` that caused 1998 merged-path discontinuities out of 2497 edges.
- **Root cause**: `bestLegPath` was not updated after the alt-bridge changed the effective departure node for the current leg
- **Solution**: After a successful alt-bridge, re-route the current leg departing from `bestFromSnapRetry` instead of `bestFromSnap`, ensuring the segment edge chain is unbroken
- **Result**: Zero merged-path discontinuities; `testGPXDataParisCannes` passes (was throwing `IllegalStateException: Edge not found with adjNode`)

## Prev-Leg Snap Fix (April 13, 2026)

### Problem
When routing an off-path leg (obs N → obs N+1), the chain from the previous leg could land on a dead-end spur node, making every route attempt from obs N to obs N+1 return an enormous detour (~122 km instead of ~7 km). The bridge pre-filter then discarded all non-spur snap candidates for obs N, leaving no valid from-node.

**Concrete failure**: segment 0, leg 8 (obs 30→31) — all 25 snap candidates for obs 30 were bridge-filtered to only three nodes near dead-end 17427691/17427692, all routing 122 km to obs 31 (threshold: 12.8 km).

### Solution: 2-Level Prev-Leg Snap Fix
Runs after the waypoint-skip fallback, before the short-spur undo:

1. **Step 1 — find a better obs-N snap**: Scan *all* snap candidates for the current from-observation (ignoring the bridge pre-filter) to find one that routes to the next observation within threshold. This yields `altFromSnap`.

2. **Step 2 — build prev-leg from-trial list**: Assemble a list of candidate from-nodes for the *previous* leg — `prevChainSnap` (the "old snap") first, then all alternative snaps for obs N-1 from `waypointAllSnapsMap`.

3. **L1 (replace 1 leg)**: If `prevChainSnap → altFromSnap` is feasible, trim the last committed leg and replace it. Sets `suitablePathFound = true`.

4. **L2 (replace 2 legs)**: If L1 fails (dead-end can't reach `altFromSnap` either), iterate over prev-leg alternatives (`prevLegFromTrials`). For the first `prevLegTrial` that routes to `altFromSnap`, check whether `prevPrevChainSnap → prevLegTrial` is also feasible. If so, trim the last **two** committed legs and replace them with the two new legs.

### New Tracking Variables
- `prevPrevChainSnap` — snap from 3 legs back (the "grandparent" chain node)
- `prevPrevIterEdgeCount` — edge count of the leg 2 iterations back

Both are maintained on every leg commit and cleared on short-spur undo.

### Result
**L2 triggered**: `17428963 → 25079 → 17429082` (143 m + 949 m), allowing obs 30→31→32 to route cleanly. Segment `end_node = 17427648` (valid bestPath splice node); full off-path segment covers 41.6 km across 72 edges. `testGPXDataParisCannes` passes.

## Code Refactoring (April 15, 2026)

### Waypoints as Filtered Positions
- **Problem**: `buildWaypointsList` stored original observation indices (`getPoint().index`) in the waypoints list, requiring a reverse map (`originalToFilteredPos`) and a snap cache (`waypointAllSnapsMap`) to translate back to filtered positions at every use site.
- **Solution**: Changed waypoints to store **filtered positions** (indices into `filteredObservations`) directly. Since `snapsPerObservationTmp` is already indexed by filtered position, all snap lookups become `snapsPerObservationTmp.get(filteredPos)` — a direct O(1) access.
- **Removed**:
  - `originalToFilteredPos` `HashMap` (was built and used in 6 call sites)
  - `waypointAllSnapsMap` `LinkedHashMap` (was built with linear scans; also handled lazy-loading for the anchor back-step)
  - `findSnapsForObs` helper (only existed to serve the lazy-loading above)
- **Logging**: Original indices for human-readable output are still derived on-demand via `filteredObservations.get(filteredPos).getPoint().index`.
- **`buildWaypointsList` simplification**: Pre-scan loops for anchor-before/after were merged with the waypoint-building loops, cutting the method from 5 loops to 2.

### `performViaWaypointRouting` Decomposition
Decomposed the ~450-line monolithic `performViaWaypointRouting` method into a 55-line thin orchestrator plus 6 focused helpers.

#### New inner class: `SegmentResult`
Holds the mutable per-segment routing state (`edges`, `startNode`, `endNode`, `spliceable`, `allLegsRouted`, `lastToSnap`) that is updated in place by salvage and splice steps.

#### Extracted methods

| Method | Single responsibility |
|---|---|
| `validateAndLogWaypointSnaps` | Fail-fast snap validation + debug logging for all waypoints |
| `buildBestPathNodeSet` | Extract the set of all nodes visited by the best path |
| `routeSegmentLegs` | Leg-chaining while loop with 4 ordered fallbacks |
| `salvagePartialSegment` | Trim a partial segment to the last best-path node after an early exit |
| `spliceSegmentBoundaries` | Walk-back (start) and walk-forward (end) bridge onto best path |
| `logSegmentGeoJson` | Build and print the GeoJSON Feature for external debug visualisation |

#### Resulting orchestrator flow
```
computeOffPathSegments → buildWaypointsList
validateAndLogWaypointSnaps   // fail-fast if any waypoint has no snaps
buildBestPathNodeSet
for each segment:
  routeSegmentLegs → SegmentResult
  if !spliceable: clear boundary nodes
  salvagePartialSegment
  spliceSegmentBoundaries
  record edges + boundary nodes
  logSegmentGeoJson
  if !allLegsRouted: break
return ViaWaypointRoutingResult
```

#### Metrics
- **Before**: ~450 lines in one method
- **After**: 55-line orchestrator + 6 helpers (each 20–90 lines)
- No functional changes; `testGPXDataParisCannes` passes.

## Walk-Back/Forward Splice Threshold Fix (April 15, 2026)

### Problem
`spliceSegmentBoundaries` used a fixed `MAX_BRIDGE_DISTANCE` (20 km) cap for walk-back and walk-forward splice bridges, while leg routing used `routingThreshold()` (2× direct GPS distance). For long off-path segments (e.g. Brest–Paris leg 0: 27,885 m), the splice bridge could exceed 20 km even though it was within the reasonable routing threshold, causing "WARNING: could not find walk-back splice for start of segment 0" and marking the segment unspliceable.

### Solution
Replaced `MAX_BRIDGE_DISTANCE` with `routingThreshold()` in both walk-back and walk-forward splice checks:
- **Walk-back**: `routingThreshold(bpSnap.getQueryPoint(), startNodePoint)` — threshold from on-path snap to segment start observation
- **Walk-forward**: `routingThreshold(endNodePoint, bpSnap.getQueryPoint())` — threshold from segment end observation to on-path snap

### Impact
Splice bridges are now accepted up to the same distance threshold used by leg routing, consistent with the rest of the via-waypoint routing pipeline. Long-distance off-path segments (like Brest–Paris) can now be successfully spliced back onto the best path.
