// SPDX-FileCopyrightText: 2024-present hkrn
// SPDX-License-Identifier: MPL

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using VRC.Dynamics;
using VRC.SDK3.Avatars.Components;
using VRC.SDK3.Dynamics.PhysBone.Components;

namespace com.github.hkrn
{
    public sealed class PhysBoneGridSearchTest
    {
        private const int SampleRate = 60;
        private const int FrameCount = 180;
        private const float SettlingTolerance = 0.005f;
        private const int ReferenceSettleWindowFrames = 15;
        private const float ReferenceSettleTolerance = 0.001f;
        private const int SegmentCountPerChain = 4;

        private static readonly float[] StiffnessScales = { 12.0f, 16.0f, 20.0f, 24.0f };
        private static readonly float[] InertiaScales = { 2.0f, 3.0f, 4.0f, 5.0f };
        private static readonly float[] GravityScales = { 2.0f, 5.0f, 10.0f, 20.0f };

        private static readonly MotionScenario[] MotionScenarios =
        {
            new MotionScenario
            {
                Name = "lateral-roll",
                Translation = new Vector3(0.14f, 0.0f, 0.0f),
                RotationDegrees = new Vector3(0.0f, 0.0f, 8.0f),
            },
            new MotionScenario
            {
                Name = "vertical-forward-tilt",
                Translation = new Vector3(0.0f, 0.06f, 0.12f),
                RotationDegrees = new Vector3(7.0f, 5.0f, 0.0f),
            },
        };

        private sealed class MotionScenario
        {
            public string Name;
            public Vector3 Translation;
            public Vector3 RotationDegrees;
            public float OutboundDuration = 0.25f;
            public float ReturnDuration = 0.20f;

            public float ImpulseEndTime => OutboundDuration + ReturnDuration;

            public Vector3 PositionAt(float time)
            {
                float amount;
                if (time < OutboundDuration)
                    amount = time / OutboundDuration;
                else if (time < ImpulseEndTime)
                    amount = 1.0f - (time - OutboundDuration) / ReturnDuration;
                else
                    amount = 0.0f;

                return Translation * amount;
            }

            public Quaternion RotationAt(float time)
            {
                if (time >= ImpulseEndTime)
                    return Quaternion.identity;

                var amount = Mathf.Sin(Mathf.PI * time / ImpulseEndTime);
                return Quaternion.Euler(RotationDegrees * amount);
            }
        }

        private sealed class ScenarioCapture
        {
            public MotionScenario Scenario;
            public GameObject Avatar;
            public List<Chain> Chains;
            public IReadOnlyList<Sample>[] References;
        }

        private sealed class Chain
        {
            public Transform[] Nodes;
            public VRCPhysBone PhysBone;
            public Vector3[] BoneAxes;
            public float[] BoneLengths;
            public Quaternion[] InitialLocalRotations;
        }

        private sealed class Sample
        {
            public float Time;
            public Vector3[] Anchors;
            public Quaternion[] ParentRotations;
            public Vector3[] ReferenceTails;
        }

        private sealed class Candidate
        {
            public int Id;
            public bool IsBaseline;
            public VrmRootExporter.SpringBoneForceScales Scales;
        }

        private sealed class Metrics
        {
            public Candidate Candidate;
            public float RmsError;
            public float MaxError;
            public float PeakReferenceDeflection;
            public float PeakCandidateDeflection;
            public float SettlingTime;
        }

        [UnityTest]
        [Category("PhysBoneGridSearch")]
        public IEnumerator CapturesVrcMotionsAndEvaluates64CandidatesAgainstProductionBaseline()
        {
            var oldCaptureFramerate = Time.captureFramerate;
            var oldFixedDeltaTime = Time.fixedDeltaTime;
            var oldTimeScale = Time.timeScale;

            yield return new EnterPlayMode();

            Time.timeScale = 1.0f;
            Time.captureFramerate = SampleRate;
            Time.fixedDeltaTime = 1.0f / SampleRate;

            var captures = new List<ScenarioCapture>(MotionScenarios.Length);
            foreach (var scenario in MotionScenarios)
            {
                var avatar = CreateFixture(out var hips, out var chains);
                var references = chains.Select(_ => new List<Sample>()).ToArray();

                for (var i = 0; i < chains.Count; i++)
                    references[i].Add(Capture(chains[i], 0.0f));

                for (var frame = 1; frame <= FrameCount; frame++)
                {
                    var time = frame / (float)SampleRate;
                    hips.localPosition = scenario.PositionAt(time);
                    hips.localRotation = scenario.RotationAt(time);
                    yield return null;

                    for (var i = 0; i < chains.Count; i++)
                        references[i].Add(Capture(chains[i], time));
                }

                Assert.That(IsReferenceSettled(references), Is.True,
                    "VRC PhysBone reference did not reach a stable terminal pose for scenario '" +
                    scenario.Name + "' during the final 0.25 seconds.");

                captures.Add(new ScenarioCapture
                {
                    Scenario = scenario,
                    Avatar = avatar,
                    Chains = chains,
                    References = references,
                });
            }

            var output = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "PhysBoneGridSearchResults"));
            if (Directory.Exists(output))
                Directory.Delete(output, true);
            Directory.CreateDirectory(output);

            WriteReference(Path.Combine(output, "reference.csv"), captures);
            var candidates = BuildCandidates();
            var results = new List<Metrics>(candidates.Count + 1);
            var baseline = new Candidate
            {
                Id = 0,
                IsBaseline = true,
                Scales = VrmRootExporter.SpringBoneForceScales.Default,
            };
            results.Add(Evaluate(baseline, captures, output));

            foreach (var candidate in candidates)
                results.Add(Evaluate(candidate, captures, output));

            WriteSummary(Path.Combine(output, "summary.csv"), results);
            WriteMetadata(Path.Combine(output, "metadata.json"), captures, candidates.Count, baseline.Scales);
            WriteBest(Path.Combine(output, "best-candidate.txt"), results);

            var peakReferenceMotion = results.Max(result => result.PeakReferenceDeflection);

            yield return new ExitPlayMode();

            Time.captureFramerate = oldCaptureFramerate;
            Time.fixedDeltaTime = oldFixedDeltaTime;
            Time.timeScale = oldTimeScale;
            // The fixtures live only in Play Mode and are removed when Play Mode exits.

            Assert.That(captures.Count, Is.EqualTo(2));
            Assert.That(captures.All(capture => capture.Chains.Count == 4), Is.True);
            Assert.That(captures.All(capture => capture.References.All(samples =>
                samples.Count == FrameCount + 1)), Is.True);
            Assert.That(candidates.Count, Is.EqualTo(64));
            Assert.That(results.Count, Is.EqualTo(65));
            Assert.That(peakReferenceMotion, Is.GreaterThan(0.0001f),
                "PhysBone did not move in Play Mode. The reference capture is unusable.");
            Assert.That(File.Exists(Path.Combine(output, "reference.csv")), Is.True);
            Assert.That(File.Exists(Path.Combine(output, "summary.csv")), Is.True);
            Assert.That(File.Exists(Path.Combine(output, "baseline.csv")), Is.True);
            Assert.That(Directory.GetFiles(output, "candidate-*.csv").Length, Is.EqualTo(64));
        }

        private static bool IsReferenceSettled(IReadOnlyList<Sample>[] references)
        {
            return references.All(samples =>
            {
                var finalTails = samples[samples.Count - 1].ReferenceTails;
                return samples.Skip(samples.Count - ReferenceSettleWindowFrames).All(sample =>
                {
                    for (var segment = 0; segment < finalTails.Length; segment++)
                    {
                        if (Vector3.Distance(sample.ReferenceTails[segment], finalTails[segment])
                            > ReferenceSettleTolerance)
                        {
                            return false;
                        }
                    }
                    return true;
                });
            });
        }

        private static GameObject CreateFixture(out Transform hips, out List<Chain> chains)
        {
            var avatar = new GameObject("PhysBoneGridSearchAvatar");
            avatar.AddComponent<Animator>();
            avatar.AddComponent<VRCAvatarDescriptor>();

            var hipsObject = new GameObject("Hips");
            hipsObject.transform.SetParent(avatar.transform, false);
            hips = hipsObject.transform;
            chains = new List<Chain>();

            // Four radial skirt panels, each represented by four consecutive spring segments.
            for (var i = 0; i < 4; i++)
            {
                var radial = Quaternion.Euler(0.0f, i * 90.0f, 0.0f) * Vector3.forward;
                var root = new GameObject("SkirtRoot_" + i);
                root.transform.SetParent(hips, false);
                root.transform.localPosition = radial * 0.08f;

                var axis = (radial * 0.12f + Vector3.down).normalized;
                var segmentLength = 0.10f + i * 0.005f;
                var nodes = new List<Transform>(SegmentCountPerChain + 1) { root.transform };
                var parent = root.transform;

                for (var segment = 0; segment < SegmentCountPerChain; segment++)
                {
                    var joint = new GameObject("SkirtJoint_" + i + "_" + segment);
                    joint.transform.SetParent(parent, false);
                    joint.transform.localPosition = axis * segmentLength;
                    nodes.Add(joint.transform);
                    parent = joint.transform;
                }

                var nodeArray = nodes.ToArray();
                var boneAxes = new Vector3[SegmentCountPerChain];
                var boneLengths = new float[SegmentCountPerChain];
                var initialLocalRotations = new Quaternion[SegmentCountPerChain];
                for (var segment = 0; segment < SegmentCountPerChain; segment++)
                {
                    var localOffset = nodeArray[segment + 1].localPosition;
                    boneAxes[segment] = localOffset.normalized;
                    boneLengths[segment] = localOffset.magnitude;
                    initialLocalRotations[segment] = nodeArray[segment].localRotation;
                }

                var physBone = root.AddComponent<VRCPhysBone>();
                physBone.version = VRCPhysBoneBase.Version.Version_1_0;
                physBone.integrationType = VRCPhysBoneBase.IntegrationType.Simplified;
                physBone.pull = 0.42f;
                physBone.spring = 0.5f;
                physBone.gravity = 0.04f;
                physBone.gravityFalloff = 0.0f;
                physBone.radius = 0.015f;
                physBone.immobile = 0.0f;

                chains.Add(new Chain
                {
                    Nodes = nodeArray,
                    PhysBone = physBone,
                    BoneAxes = boneAxes,
                    BoneLengths = boneLengths,
                    InitialLocalRotations = initialLocalRotations,
                });
            }

            return avatar;
        }

        private static Sample Capture(Chain chain, float time)
        {
            var segmentCount = chain.BoneLengths.Length;
            var sample = new Sample
            {
                Time = time,
                Anchors = new Vector3[segmentCount],
                ParentRotations = new Quaternion[segmentCount],
                ReferenceTails = new Vector3[segmentCount],
            };

            for (var segment = 0; segment < segmentCount; segment++)
            {
                var head = chain.Nodes[segment];
                var tail = chain.Nodes[segment + 1];
                var parent = head.parent;

                sample.Anchors[segment] = head.position;
                sample.ParentRotations[segment] = parent != null ? parent.rotation : Quaternion.identity;
                sample.ReferenceTails[segment] = tail.position;
            }

            return sample;
        }

        private static List<Candidate> BuildCandidates()
        {
            var output = new List<Candidate>(64);
            var id = 1;
            foreach (var stiffness in StiffnessScales)
            foreach (var inertia in InertiaScales)
            foreach (var gravity in GravityScales)
            {
                output.Add(new Candidate
                {
                    Id = id++,
                    Scales = new VrmRootExporter.SpringBoneForceScales(stiffness, inertia, gravity),
                });
            }
            return output;
        }

        private static Metrics Evaluate(
            Candidate candidate,
            IReadOnlyList<ScenarioCapture> captures,
            string outputDirectory)
        {
            var metrics = new Metrics { Candidate = candidate, SettlingTime = -1.0f };
            var csv = new StringBuilder();
            csv.AppendLine("scenario,frame,time,chain,segment,referenceX,referenceY,referenceZ,candidateX,candidateY,candidateZ,errorMeters");

            double squaredError = 0.0;
            var sampleCount = 0;
            var maxError = 0.0f;
            var peakReference = 0.0f;
            var peakCandidate = 0.0f;
            List<Vector3>[][][] candidateTracks = new List<Vector3>[][][captures.Count];

            for (var scenarioIndex = 0; scenarioIndex < captures.Count; scenarioIndex++)
            {
                var capture = captures[scenarioIndex];
                var scenarioTracks = new List<Vector3>[capture.Chains.Count][];

                for (var chainIndex = 0; chainIndex < capture.Chains.Count; chainIndex++)
                {
                    var chain = capture.Chains[chainIndex];
                    var samples = capture.References[chainIndex];
                    var segmentCount = chain.BoneLengths.Length;
                    var physBone = chain.PhysBone;
                    var force = VrmRootExporter.ConvertSpringBoneForces(
                        VRCPhysBoneBase.Version.Version_1_0,
                        VRCPhysBoneBase.IntegrationType.Simplified,
                        physBone.pull, physBone.spring, physBone.stiffness,
                        physBone.gravity, physBone.gravityFalloff, candidate.Scales);

                    var gravityDirection = new Vector3(force.GravityDir.X, force.GravityDir.Y, force.GravityDir.Z);
                    var currentTails = (Vector3[])samples[0].ReferenceTails.Clone();
                    var previousTails = (Vector3[])currentTails.Clone();
                    var tracks = new List<Vector3>[segmentCount];

                    for (var segment = 0; segment < segmentCount; segment++)
                        tracks[segment] = new List<Vector3>(samples.Count) { currentTails[segment] };

                    for (var frame = 1; frame < samples.Count; frame++)
                    {
                        var sample = samples[frame];
                        var deltaTime = sample.Time - samples[frame - 1].Time;
                        var headPosition = sample.Anchors[0];
                        var parentWorldRotation = sample.ParentRotations[0];

                        for (var segment = 0; segment < segmentCount; segment++)
                        {
                            var boneAxis = chain.BoneAxes[segment];
                            var boneLength = chain.BoneLengths[segment];
                            var initialLocalRotation = chain.InitialLocalRotations[segment];
                            var restWorldRotation = parentWorldRotation * initialLocalRotation;
                            var restDirection = restWorldRotation * boneAxis;

                            // VRMC_springBone 1.0 reference Verlet update for this joint pair.
                            var inertia = (currentTails[segment] - previousTails[segment])
                                * (1.0f - force.DragForce);
                            var stiffness = restDirection * (deltaTime * force.Stiffness);
                            var gravity = gravityDirection * (deltaTime * force.GravityPower);
                            var nextTail = currentTails[segment] + inertia + stiffness + gravity;

                            var fromHead = nextTail - headPosition;
                            if (fromHead.sqrMagnitude > Mathf.Epsilon)
                                nextTail = headPosition + fromHead.normalized * boneLength;

                            // Apply the head-node rotation before evaluating the next pair.
                            // The next pair inherits this updated world rotation as its parent.
                            var toDirection = nextTail - headPosition;
                            if (toDirection.sqrMagnitude > Mathf.Epsilon)
                            {
                                var toLocal = Quaternion.Inverse(restWorldRotation) * toDirection.normalized;
                                var localRotation = initialLocalRotation
                                    * Quaternion.FromToRotation(boneAxis, toLocal);
                                parentWorldRotation = parentWorldRotation * localRotation;
                            }
                            else
                            {
                                parentWorldRotation = restWorldRotation;
                            }

                            previousTails[segment] = currentTails[segment];
                            currentTails[segment] = nextTail;
                            tracks[segment].Add(nextTail);

                            var error = Vector3.Distance(nextTail, sample.ReferenceTails[segment]);
                            squaredError += error * error;
                            sampleCount++;
                            maxError = Mathf.Max(maxError, error);

                            var referenceRestDirection = sample.ParentRotations[segment]
                                * initialLocalRotation * boneAxis;
                            var referenceRestTail = sample.Anchors[segment]
                                + referenceRestDirection * boneLength;
                            peakReference = Mathf.Max(peakReference,
                                Vector3.Distance(sample.ReferenceTails[segment], referenceRestTail));
                            var candidateRestTail = headPosition + restDirection * boneLength;
                            peakCandidate = Mathf.Max(peakCandidate,
                                Vector3.Distance(nextTail, candidateRestTail));

                            csv.Append(capture.Scenario.Name).Append(',')
                                .Append(frame.ToString(CultureInfo.InvariantCulture)).Append(',')
                                .Append(F(sample.Time)).Append(',')
                                .Append(chainIndex.ToString(CultureInfo.InvariantCulture)).Append(',')
                                .Append(segment.ToString(CultureInfo.InvariantCulture)).Append(',')
                                .Append(F(sample.ReferenceTails[segment].x)).Append(',')
                                .Append(F(sample.ReferenceTails[segment].y)).Append(',')
                                .Append(F(sample.ReferenceTails[segment].z)).Append(',')
                                .Append(F(nextTail.x)).Append(',').Append(F(nextTail.y)).Append(',')
                                .Append(F(nextTail.z)).Append(',').Append(F(error)).AppendLine();

                            headPosition = nextTail;
                        }
                    }

                    scenarioTracks[chainIndex] = tracks;
                }

                candidateTracks[scenarioIndex] = scenarioTracks;
            }

            metrics.RmsError = sampleCount == 0 ? 0.0f : Mathf.Sqrt((float)(squaredError / sampleCount));
            metrics.MaxError = maxError;
            metrics.PeakReferenceDeflection = peakReference;
            metrics.PeakCandidateDeflection = peakCandidate;
            metrics.SettlingTime = FindSettlingTime(captures, candidateTracks);
            var fileName = candidate.IsBaseline ? "baseline.csv"
                : "candidate-" + candidate.Id.ToString("D3", CultureInfo.InvariantCulture) + ".csv";
            File.WriteAllText(Path.Combine(outputDirectory, fileName),
                csv.ToString(), new UTF8Encoding(false));
            return metrics;
        }

        private static float FindSettlingTime(
            IReadOnlyList<ScenarioCapture> captures,
            List<Vector3>[][][] tracks)
        {
            var worstSettlingTime = 0.0f;
            for (var scenarioIndex = 0; scenarioIndex < captures.Count; scenarioIndex++)
            {
                var scenario = captures[scenarioIndex];
                var settlingTime = FindScenarioSettlingTime(
                    scenario.References,
                    tracks[scenarioIndex],
                    scenario.Scenario.ImpulseEndTime);
                if (settlingTime < 0.0f)
                    return -1.0f;

                worstSettlingTime = Mathf.Max(worstSettlingTime, settlingTime);
            }

            return worstSettlingTime;
        }

        private static float FindScenarioSettlingTime(
            IReadOnlyList<Sample>[] references,
            List<Vector3>[][] tracks,
            float impulseEndTime)
        {
            var first = Mathf.CeilToInt(impulseEndTime * SampleRate);
            for (var frame = first; frame <= FrameCount; frame++)
            {
                var settled = true;
                for (var chainIndex = 0; chainIndex < references.Length && settled; chainIndex++)
                {
                    var samples = references[chainIndex];
                    var finalTails = samples[samples.Count - 1].ReferenceTails;
                    for (var segment = 0; segment < finalTails.Length && settled; segment++)
                    {
                        var referenceTail = finalTails[segment];
                        for (var next = frame; next < samples.Count; next++)
                        {
                            if (Vector3.Distance(tracks[chainIndex][segment][next], referenceTail)
                                > SettlingTolerance)
                            {
                                settled = false;
                                break;
                            }
                        }
                    }
                }

                if (settled)
                    return frame / (float)SampleRate - impulseEndTime;
            }

            return -1.0f;
        }

        private static void WriteReference(string path, IReadOnlyList<ScenarioCapture> captures)
        {
            var csv = new StringBuilder();
            csv.AppendLine("scenario,frame,time,chain,segment,anchorX,anchorY,anchorZ,parentQX,parentQY,parentQZ,parentQW,tailX,tailY,tailZ");
            for (var scenarioIndex = 0; scenarioIndex < captures.Count; scenarioIndex++)
            {
                var capture = captures[scenarioIndex];
                for (var chain = 0; chain < capture.References.Length; chain++)
                for (var frame = 0; frame < capture.References[chain].Count; frame++)
                for (var segment = 0; segment < capture.References[chain][frame].ReferenceTails.Length; segment++)
                {
                    var sample = capture.References[chain][frame];
                    var anchor = sample.Anchors[segment];
                    var parentRotation = sample.ParentRotations[segment];
                    var tail = sample.ReferenceTails[segment];

                    csv.Append(capture.Scenario.Name).Append(',')
                        .Append(frame.ToString(CultureInfo.InvariantCulture)).Append(',')
                        .Append(F(sample.Time)).Append(',')
                        .Append(chain.ToString(CultureInfo.InvariantCulture)).Append(',')
                        .Append(segment.ToString(CultureInfo.InvariantCulture)).Append(',')
                        .Append(F(anchor.x)).Append(',').Append(F(anchor.y)).Append(',')
                        .Append(F(anchor.z)).Append(',').Append(F(parentRotation.x)).Append(',')
                        .Append(F(parentRotation.y)).Append(',').Append(F(parentRotation.z)).Append(',')
                        .Append(F(parentRotation.w)).Append(',').Append(F(tail.x)).Append(',')
                        .Append(F(tail.y)).Append(',').Append(F(tail.z)).AppendLine();
                }
            }
            File.WriteAllText(path, csv.ToString(), new UTF8Encoding(false));
        }

        private static void WriteSummary(string path, IReadOnlyList<Metrics> results)
        {
            var csv = new StringBuilder();
            csv.AppendLine("candidateId,stiffnessScale,inertiaScale,gravityScale,rmsErrorMeters,maxErrorMeters,peakReferenceDeflectionMeters,peakCandidateDeflectionMeters,settlingTimeSeconds");
            foreach (var item in results)
            {
                csv.Append(item.Candidate.IsBaseline ? "baseline" : item.Candidate.Id.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(F(item.Candidate.Scales.Stiffness)).Append(',').Append(F(item.Candidate.Scales.Inertia)).Append(',')
                    .Append(F(item.Candidate.Scales.Gravity)).Append(',').Append(F(item.RmsError)).Append(',')
                    .Append(F(item.MaxError)).Append(',').Append(F(item.PeakReferenceDeflection)).Append(',')
                    .Append(F(item.PeakCandidateDeflection)).Append(',').Append(F(item.SettlingTime)).AppendLine();
            }
            File.WriteAllText(path, csv.ToString(), new UTF8Encoding(false));
        }

        private static void WriteMetadata(
            string path,
            IReadOnlyList<ScenarioCapture> captures,
            int candidateCount,
            VrmRootExporter.SpringBoneForceScales baselineScales)
        {
            var chainCount = captures[0].Chains.Count;
            var segmentCount = captures[0].Chains[0].BoneLengths.Length;
            var pairsPerScenario = captures[0].Chains.Sum(chain => chain.BoneLengths.Length);
            var totalScenarioJointPairs = captures.Sum(capture =>
                capture.Chains.Sum(chain => chain.BoneLengths.Length));
            var physBone = captures[0].Chains[0].PhysBone;

            var json = new StringBuilder();
            json.AppendLine("{");
            json.Append("  \"unityVersion\": \"").Append(Application.unityVersion).AppendLine("\",");
            json.Append("  \"sampleRateHz\": ").Append(SampleRate.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"frameCount\": ").Append(FrameCount.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"scenarioCount\": ").Append(captures.Count.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.AppendLine("  \"scenarios\": [");
            for (var i = 0; i < captures.Count; i++)
            {
                json.Append("    \"").Append(captures[i].Scenario.Name).Append("\"");
                json.AppendLine(i + 1 < captures.Count ? "," : string.Empty);
            }
            json.AppendLine("  ],");
            json.Append("  \"chainCountPerScenario\": ").Append(chainCount.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"segmentCountPerChain\": ").Append(segmentCount.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"simulatedJointPairCountPerScenario\": ").Append(pairsPerScenario.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"totalScenarioJointPairCount\": ").Append(totalScenarioJointPairs.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"candidateCount\": ").Append(candidateCount.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.AppendLine("  \"baselineIncluded\": true,");
            json.Append("  \"evaluationCount\": ").Append((candidateCount + 1).ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.AppendLine("  \"baselineScales\": {");
            json.Append("    \"stiffness\": ").Append(F(baselineScales.Stiffness)).AppendLine(",");
            json.Append("    \"inertia\": ").Append(F(baselineScales.Inertia)).AppendLine(",");
            json.Append("    \"gravity\": ").Append(F(baselineScales.Gravity)).AppendLine();
            json.AppendLine("  },");
            json.AppendLine("  \"physBone\": {");
            json.AppendLine("    \"version\": \"1.0\",");
            json.AppendLine("    \"integrationType\": \"Simplified\",");
            json.Append("    \"pull\": ").Append(F(physBone.pull)).AppendLine(",");
            json.Append("    \"spring\": ").Append(F(physBone.spring)).AppendLine(",");
            json.Append("    \"gravity\": ").Append(F(physBone.gravity)).AppendLine(",");
            json.Append("    \"gravityFalloff\": ").Append(F(physBone.gravityFalloff)).AppendLine();
            json.AppendLine("  },");
            json.AppendLine("  \"scaleAxes\": {");
            json.AppendLine("    \"stiffness\": [12, 16, 20, 24],");
            json.AppendLine("    \"inertia\": [2, 3, 4, 5],");
            json.AppendLine("    \"gravity\": [2, 5, 10, 20]");
            json.AppendLine("  },");
            json.AppendLine("  \"referenceAlgorithm\": \"https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_springBone-1.0/README.md#update-procedure\",");
            json.AppendLine("  \"settlingCriterion\": \"Candidate tails remain within 0.005m of the captured VRC terminal tail poses for all remaining samples in every scenario.\",");
            json.AppendLine("  \"notes\": \"Synthetic four-panel skirt fixture with four consecutive measured segments per chain and two independent body-motion scenarios. Candidate motion applies each updated head-node rotation before evaluating the next VRMC_springBone 1.0 joint pair. The fixtures use unit scale and world space. No colliders, limits, or non-unit scale cases are included.\",");
            json.AppendLine("  \"artifacts\": [\"reference.csv\", \"baseline.csv\", \"candidate-001.csv through candidate-064.csv\", \"summary.csv\"]");
            json.AppendLine("}");
            File.WriteAllText(path, json.ToString(), new UTF8Encoding(false));
        }

        private static void WriteBest(string path, IReadOnlyList<Metrics> results)
        {
            var settled = results.Where(item => item.SettlingTime >= 0.0f).ToList();
            var candidates = settled.Count > 0 ? settled : results.ToList();
            var best = candidates.OrderBy(item => item.RmsError).First();
            var text = new StringBuilder();
            text.AppendLine(settled.Count > 0
                ? $"Best settled candidate by RMS position error: {CandidateLabel(best.Candidate)}"
                : $"No candidate met the terminal-pose tolerance; best RMS candidate: {CandidateLabel(best.Candidate)}");
            text.AppendLine($"Stiffness scale: {F(best.Candidate.Scales.Stiffness)}");
            text.AppendLine($"Inertia scale: {F(best.Candidate.Scales.Inertia)}");
            text.AppendLine($"Gravity scale: {F(best.Candidate.Scales.Gravity)}");
            text.AppendLine($"RMS error: {F(best.RmsError)} m");
            text.AppendLine($"Maximum error: {F(best.MaxError)} m");
            text.AppendLine($"Settling time: {F(best.SettlingTime)} s (-1 means not settled)");
            text.AppendLine($"Candidates meeting settling tolerance: {settled.Count}/{results.Count} (64 grid candidates plus production baseline)");
            File.WriteAllText(path, text.ToString(), new UTF8Encoding(false));
        }

        private static string CandidateLabel(Candidate candidate)
        {
            return candidate.IsBaseline ? "production baseline" : $"#{candidate.Id:D3}";
        }

        private static string F(float value)
        {
            return value.ToString("0.######", CultureInfo.InvariantCulture);
        }
    }
}
