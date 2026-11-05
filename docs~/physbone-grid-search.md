# PhysBone Force Grid Search

The `PhysBoneGridSearchTest` test captures a deterministic VRC PhysBone reference motion in Unity Play Mode, evaluates a grid of VRM SpringBone force scales against the captured motion, and writes every candidate trajectory to disk.

## Scope

The initial calibration target is deliberately narrow:

- PhysBone version: 1.0
- Integration type: Simplified
- Scenario: a synthetic four-panel skirt moving sideways and returning to rest
- Sampling: 60 Hz for 180 frames
- Spring representation: one measured Root-to-Mid segment per PhysBone chain
- Colliders and limits: not included in this first fixture

This is a calibration harness, not a complete equivalence test for arbitrary avatars. It currently evaluates the VRMC_springBone 1.0 reference update procedure for one segment per chain. Multi-joint bending, collisions, spring limits, and additional body-motion scenarios should be added in later iterations.

## Grid

The test evaluates all combinations of the following scales:

| Scale | Candidate values |
| --- | --- |
| Stiffness | 1, 2, 4, 8 |
| Inertia | 0.25, 0.5, 0.75, 1.0 |
| Gravity | 2, 5, 10, 20 |

The full grid contains 64 candidates.

The default production conversion uses Stiffness=4, Inertia=1, and Gravity=20. These are calibration scales, not constants prescribed by VRChat or the VRM specification. The default overload remains unchanged for normal exports.

## Running

Run the Unity EditMode test assembly `com.github.hkrn.NDMFVRMExporterTests`. The test enters Play Mode itself to capture the VRC PhysBone reference trajectory.

GameCI requires a valid Unity license. Configure the repository Actions secret `UNITY_LICENSE` with the Unity license file content before expecting the test runner to start. The current workflow also accepts `UNITY_EMAIL` and `UNITY_PASSWORD` for the GameCI activation flow. Without a license, package setup can succeed but the Unity test runner will stop before compiling or executing the test.

The GameCI workflow runs this test on pushes to `develop` and `feature/physbone-grid-search`, and for pull requests targeting `develop` or `main`. The generated results are uploaded as the `PhysBone grid search results` workflow artifact.

## Output

The test writes `PhysBoneGridSearchResults/` at the Unity project root:

- `metadata.json`: Unity version, fixture settings, sampling rate, candidate axes, and reference algorithm.
- `reference.csv`: captured VRC anchor and tail positions for every frame and chain.
- `candidate-001.csv` through `candidate-064.csv`: each candidate's tail trajectory and per-sample position error.
- `summary.csv`: RMS position error, maximum position error, peak deflection, and settling time for every candidate.
- `best-candidate.txt`: the candidate with the lowest RMS position error.

A settling time of `-1` means the candidate did not remain within the 5 mm rest-pose tolerance for the remainder of the capture window.

## Interpretation

RMS position error is the primary ranking metric. The best candidate should not be adopted automatically: inspect maximum error, peak deflection, and settling time as well. A candidate that fits this single synthetic motion may overfit the fixture.

The reference trajectory is captured from the VRC SDK PhysBone component running in Unity Play Mode. Candidate trajectories are produced with the VRMC_springBone 1.0 reference update equation. This isolates the parameter mapping from differences between VRM runtime implementations, but it does not yet compare a complete exported VRM inside a specific VRM viewer.

Reference algorithm: [VRMC_springBone 1.0 — Update procedure](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_springBone-1.0/README.md#update-procedure)
