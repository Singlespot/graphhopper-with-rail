# Railway Map Matching Objectives

## Primary Objectives
1. **All observations should be on final path**
   - Every GPS observation must be matched to the resulting path
   - No observation should be left unmatched or ignored
   - Observation indexes must be valid (positive integers, not -1)

2. **Final path has to have no detours**
   - The path should be the most direct route through the observations
   - No unnecessary loops or detours
   - Must follow the railway network topology efficiently

## Relevant test
- `testFullGPXTrackMatching` in RailwayMapMatchingTest.java
- `testGPXDataParisCannes` in RailwayMapMatchingTest.java

## Implementation Strategies

### Direct Path Merging
- Build bestPath for observations that are already on a good route
- Route individual segments for off-path observations
- Merge bestPath with routed segments directly
- **Pros**: Faster, bypasses expensive Viterbi computation
- **Cons**: Risk of node reference issues, must ensure compatibility, achieved through using same QueryGraph

## Key Constraints
- Must complete within 60 seconds (test timeout)
- Must handle all observations efficiently
- Must avoid mixing edges from different routing contexts
- Must ensure all node references are valid

## Success Criteria
- Test passes with all observation indexes valid
- Path includes all observations without detours
- Performance meets 60-second timeout requirement
