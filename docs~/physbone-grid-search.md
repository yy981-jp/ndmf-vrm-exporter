# PhysBone Force Grid Search

The `PhysBoneGridSearchTest` test captures a deterministic VRC PhysBone reference motion in Unity Play Mode, evaluates a grid of VRM SpringBone force scales against the captured motion, and writes every candidate trajectory to disk.

## Scope

The current calibration fixture covers the following:

- PhysBone version: 1.0
- Integration type: Simplified
- Scenarios: a lateral translation with roll, and a vertical/forward translation with pitch and yaw
- Sampling: 60 Hz for 180 frames per scenario
- Spring representation: four consecutive measured segments per PhysBone chain
- Chains: four radial skirt-like chains per scenario, with slightly different segment lengths
- Total simulated joint pairs: 16 per scenario and 32 across both scenario captures
- Colliders and limits: not included in this fixture

This is a calibration harness, not a complete equivalence test for arbitrary avatars. It evaluates four consecutive joint pairs per chain and propagates each simulated head-joint rotation before evaluating the next pair, following the root-to-descendant update order in the VRMC_springBone 1.0 reference algorithm. The fixtures use unit scale and world-space evaluation. They still omit colliders, spring limits, center-space evaluation, per-joint parameter curves, non-unit scale, PhysBone 1.0 Advanced, and PhysBone 1.1.

## Grid

The test evaluates all combinations of the following scales:

| Scale | Candidate values |
| --- | --- |
| Stiffness | 4, 8, 12, 16 |
| Inertia | 1, 2, 3, 4 |
| Gravity | 2, 5, 10, 20 |

The full grid contains 64 candidates.

The production conversion uses Stiffness=8, Inertia=3, and Gravity=5 for PhysBone 1.0 / Simplified, based on the current multi-scenario grid search. PhysBone 1.0 / Advanced and PhysBone 1.1 retain the previous fallback scales (4, 1, 20) pending separate calibration. These are exporter calibration scales, not values prescribed by VRChat or the VRM specification.

The PhysBone 1.0 / Simplified production default is evaluated separately on every run as `baseline.csv`; it is also present as a matching grid point in the current lower-side sweep so the test can assert that both evaluation paths produce the same metrics. The summary identifies the separate evaluation row as `baseline`. Each run therefore evaluates 64 grid candidates plus one production baseline (65 evaluations total). Candidate IDs are local to one grid definition and must not be compared across different search ranges. The metadata records the number of grid candidates as `candidateCount` and the total number of evaluations as `evaluationCount`.

## Running

Run the Unity EditMode test assembly `com.github.hkrn.NDMFVRMExporterTests`. The calibration test uses Unity Test Framework `UnitySetUp` to enter Play Mode before its coroutine body, captures both VRC PhysBone reference trajectories, and uses `UnityTearDown` to return to Edit Mode even when the test fails. This keeps the EditMode-only test assembly from running a coroutine across the Play Mode domain reload.

GameCI requires three repository-level Actions secrets for this Personal-license flow: `UNITY_LICENSE` (the full contents of the activated `.ulf` file), `UNITY_EMAIL` (the Unity account email), and `UNITY_PASSWORD` (the Unity account password). The test runner extracts the license serial from `UNITY_LICENSE`, then activates that serial inside the CI container using `UNITY_EMAIL` and `UNITY_PASSWORD`. The `.ulf` content alone is not sufficient for this version of the GameCI runner. Add all three under Settings → Secrets and variables → Actions → Repository secrets. Do not put credentials in workflow files or commit them. If any required secret is missing or unavailable to the workflow, package setup can succeed but the Unity test runner will stop before compiling or executing the test.

The GameCI workflow runs this test on pushes to `develop` and `feature/physbone-grid-search`, and for pull requests targeting `develop` or `main`. The generated results are uploaded as the `PhysBone grid search results` workflow artifact.

## Output

The test writes `PhysBoneGridSearchResults/` at the Unity project root:

- `metadata.json`: Unity version, both motion scenarios and their parameters, fixture settings, sampling rate, candidate axes, production-baseline scales, and reference algorithm.
- `reference.csv`: captured VRC anchor and tail positions for every scenario, frame, chain, and segment. It includes a `scenario` column.
- `baseline.csv`: production-default trajectory and per-sample position error for both scenarios.
- `candidate-001.csv` through `candidate-064.csv`: each grid candidate's tail trajectory and per-sample position error across both scenarios, including the scenario and segment columns.
- `summary.csv`: aggregate RMS position error, maximum position error, peak deflection, and worst-case settling time for all 64 grid candidates plus the `baseline` row. The aggregate RMS uses every sampled pair in both scenarios; settling requires every scenario to meet the terminal-pose criterion.
- `scenario-summary.csv`: per-candidate and per-scenario RMS position error, maximum position error, peak deflection, and settling time. Use this to spot candidates that perform well overall but poorly in one motion scenario.
- `best-candidate.txt`: the lowest-RMS evaluation that meets the terminal-pose settling criterion, or the lowest-RMS evaluation with an explicit warning if none settle. It reports grid settling counts separately from the production baseline's settling result.

For each scenario, settling time is measured from the end of that scenario's body impulse. It is the first time at which every candidate tail remains within 5 mm of its corresponding captured VRC terminal tail position for all remaining samples. The reported settling time is the slower of the two scenario results, and is `-1` if either scenario fails to settle. Each VRC reference must remain within 1 mm of its terminal pose during the final 0.25 seconds; otherwise the calibration test fails.

## Interpretation

RMS position error is the primary ranking metric, but the reported best candidate is selected from evaluations that also meet the 5 mm terminal-pose settling criterion in both scenarios. Inspect maximum error and peak deflection as well. A candidate that fits these two synthetic motions may still overfit the fixture.

The initial one-segment sweep is retained only as a historical baseline: it selected candidate #063 (Stiffness=8, Inertia=1, Gravity=10) among settled candidates. Its RMS and maximum errors are not directly comparable with the current two-segment sweep because the measured joint-pair set has changed.

The last completed two-segment, single-motion run before the expanded fixture was GameCI Run #13. It used Stiffness `[4, 8, 12, 16]`, Inertia `[1, 2, 3, 4]`, and Gravity `[2, 5, 10, 20]`. Its best settled candidate was #058 (16, 3, 5), with RMS error 0.002594 m, maximum error 0.016327 m, and settling time 0.15 s; 36 of 64 candidates settled. The production default `(4, 1, 20)` was candidate #004 in that previous grid, with RMS error 0.028268 m and no settling result. Those measurements are historical and are not directly comparable with the new four-segment, two-scenario fixture. Results from the expanded fixture are pending a new Unity test run.

### Expanded-grid and multi-scenario results

The expanded upper-side grid was first checked on the previous two-segment, single-motion fixture in [GameCI Run 42196746316](https://github.com/yy981-jp/ndmf-vrm-exporter/actions/runs/42196746316) (commit `9407c91`): 193 / 193 EditMode tests passed. Its best settled grid candidate was #022 (Stiffness=16, Inertia=3, Gravity=5), RMS 0.002594 m, maximum error 0.016327 m, settling time 0.15 s; 56 / 64 grid candidates settled. The separate production baseline had RMS 0.028268 m, maximum error 0.124441 m, and did not settle. The best scale tuple matched the earlier Run #13 result, although the candidate ID changed with the grid ordering.

The longer-chain, two-scenario fixture then passed in [GameCI Run 42197409067](https://github.com/yy981-jp/ndmf-vrm-exporter/actions/runs/42197409067) (commit `d7fe6e9`): 193 / 193 EditMode tests passed. On the upper-side grid, the best settled candidate was #002 (Stiffness=12, Inertia=2, Gravity=5), aggregate RMS 0.006676 m, maximum error 0.052202 m, and worst-case settling time 0.166667 s. Only 24 / 64 grid candidates settled. The production baseline had aggregate RMS 0.028647 m, maximum error 0.188942 m, and did not settle.

The per-scenario artifact from [GameCI Run 42197727655](https://github.com/yy981-jp/ndmf-vrm-exporter/actions/runs/42197727655) contains 130 data rows (65 evaluations × 2 scenarios). For candidate #002:

| Scenario | RMS error | Maximum error | Settling time |
| --- | ---: | ---: | ---: |
| lateral-roll | 7.492 mm | 52.202 mm | 0.166667 s |
| vertical-forward-tilt | 5.746 mm | 46.052 mm | 0.166667 s |

For the production baseline, lateral-roll measured 32.391 mm RMS / 188.942 mm maximum error, and vertical-forward-tilt measured 24.333 mm RMS / 122.635 mm maximum error; neither scenario settled.

The previous short-chain best tuple (16, 3, 5) became candidate #022 in the longer-chain sweep and produced 0.015602 m RMS, 0.124242 m maximum error, and 0.333333 s settling. This is evidence that the best mapping depends on chain depth and motion scenario. The upper-side grid put the best stiffness and inertia values at their lower search bounds, so the current search axes were shifted downward to Stiffness `[4, 8, 12, 16]` and Inertia `[1, 2, 3, 4]`. In the resulting four-segment, two-scenario run (GameCI Run 42208638241), candidate #026 `(8, 3, 5)` achieved 0.003227 m RMS error, 0.024047 m maximum error, and 0.366667 s settling; 24 / 64 grid candidates settled. The former production baseline `(4, 1, 20)` measured 0.028647 m RMS error and 0.188942 m maximum error, and did not settle.

Based on this result, `(8, 3, 5)` is now the provisional production default for PhysBone 1.0 / Simplified only. PhysBone 1.0 / Advanced and PhysBone 1.1 retain `(4, 1, 20)` until they receive separate calibration. The synthetic fixture does not establish universal equivalence; real-avatar, collider, scale, and independent-runtime validation remain necessary.

The reference trajectory is captured from the VRC SDK PhysBone component running in Unity Play Mode. Candidate trajectories are produced with the non-normative VRMC_springBone 1.0 reference update procedure: Verlet integration, length constraint, then head-node rotation, processing each chain from root to descendant. This isolates the parameter mapping from differences between VRM runtime implementations, but it does not yet compare a complete exported VRM inside a specific VRM viewer.

Reference algorithm: [VRMC_springBone 1.0 — Update procedure](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_springBone-1.0/README.md#update-procedure)
