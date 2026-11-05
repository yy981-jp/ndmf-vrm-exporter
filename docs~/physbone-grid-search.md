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

GameCI requires three repository-level Actions secrets for this Personal-license flow: `UNITY_LICENSE` (the full contents of the activated `.ulf` file), `UNITY_EMAIL` (the Unity account email), and `UNITY_PASSWORD` (the Unity account password). The test runner extracts the license serial from `UNITY_LICENSE`, then activates that serial inside the CI container using `UNITY_EMAIL` and `UNITY_PASSWORD`. The `.ulf` content alone is not sufficient for this version of the GameCI runner. Add all three under Settings → Secrets and variables → Actions → Repository secrets. Do not put credentials in workflow files or commit them. If any required secret is missing or unavailable to the workflow, package setup can succeed but the Unity test runner will stop before compiling or executing the test.

The GameCI workflow runs this test on pushes to `develop` and `feature/physbone-grid-search`, and for pull requests targeting `develop` or `main`. The generated results are uploaded as the `PhysBone grid search results` workflow artifact.

## Output

The test writes `PhysBoneGridSearchResults/` at the Unity project root:

- `metadata.json`: Unity version, fixture settings, sampling rate, candidate axes, and reference algorithm.
- `reference.csv`: captured VRC anchor and tail positions for every frame and chain.
- `candidate-001.csv` through `candidate-064.csv`: each candidate's tail trajectory and per-sample position error.
- `summary.csv`: RMS position error, maximum position error, peak deflection, and settling time for every candidate.
- `best-candidate.txt`: the lowest-RMS candidate that meets the terminal-pose settling criterion, or the lowest-RMS candidate with an explicit warning if none settle.

A settling time is measured from the end of the body impulse. It is the first time at which every candidate tail remains within 5 mm of its corresponding captured VRC terminal tail position for all remaining samples. The reference itself must remain within 1 mm of its terminal pose during the final 0.25 seconds; otherwise the calibration test fails. A settling time of `-1` means the candidate did not meet this terminal-pose tolerance.

## Interpretation

RMS position error is the primary ranking metric, but the reported best candidate is selected from candidates that also meet the 5 mm terminal-pose settling criterion. Inspect maximum error and peak deflection as well. A candidate that fits this single synthetic motion may overfit the fixture.

The first 64-candidate sweep (Unity 2022.3.22f1) produced a preliminary best settled candidate of Stiffness=8, Inertia=1, Gravity=10 (candidate #063): RMS error 0.010496 m and maximum error 0.035343 m. The existing production scales (4, 1, 20; candidate #048) produced RMS error 0.015965 m and maximum error 0.054523 m in the same fixture. The preliminary candidate therefore reduced RMS error by about 34.3% and maximum error by about 35.2%. Candidate #064 (8, 1, 20) had a slightly lower raw RMS error (0.010034 m), but ended 5.064 mm from the VRC terminal pose and therefore missed the 5 mm criterion. These results are specific to this one-segment synthetic fixture and are not sufficient to change production defaults.

The reference trajectory is captured from the VRC SDK PhysBone component running in Unity Play Mode. Candidate trajectories are produced with the VRMC_springBone 1.0 reference update equation. This isolates the parameter mapping from differences between VRM runtime implementations, but it does not yet compare a complete exported VRM inside a specific VRM viewer.

Reference algorithm: [VRMC_springBone 1.0 — Update procedure](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_springBone-1.0/README.md#update-procedure)
