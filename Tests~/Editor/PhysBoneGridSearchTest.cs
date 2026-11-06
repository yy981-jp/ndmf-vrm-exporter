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
        private const float ImpulseEndTime = 0.45f;
        private const float SettlingTolerance = 0.005f;
        private const int ReferenceSettleWindowFrames = 15;
        private const float ReferenceSettleTolerance = 0.001f;
        private const int SegmentCountPerChain = 2;

        private static readonly float[] StiffnessScales = { 4.0f, 8.0f, 12.0f, 16.0f };
        private static readonly float[] InertiaScales = { 1.0f, 2.0f, 3.0f, 4.0f };
        private static readonly float[] GravityScales = { 2.0f, 5.0f, 10.0f, 20.0f };

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
        public IEnumerator CapturesVrcMotionAndEvaluatesAll64Candidates()
        {
            var oldCaptureFramerate = Time.captureFramerate;
            var oldFixedDeltaTime = Time.fixedDeltaTime;
            var oldTimeScale = Time.timeScale;

            yield return new EnterPlayMode();

            CreateFixture(out var hips, out var chains);
            var references = chains.Select(_ => new List<Sample>()).ToArray();
            Time.timeScale = 1.0f;
            Time.captureFramerate = SampleRate;
            Time.fixedDeltaTime = 1.0f / SampleRate;

            for (var i = 0; i < chains.Count; i++)
                references[i].Add(Capture(chains[i], hips, 0.0f));

            for (var frame = 1; frame <= FrameCount; frame++)
            {
                var time = frame / (float)SampleRate;
                hips.localPosition = BodyPosition(time);
                hips.localRotation = BodyRotation(time);
                yield return null;

                for (var i = 0; i < chains.Count; i++)
                    references[i].Add(Capture(chains[i], hips, time));
            }

            Assert.That(references.All(samples =>
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
            }), Is.True,
                "VRC PhysBone reference did not reach a stable terminal pose for every joint pair during the final 0.25 seconds.");
            var output = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "PhysBoneGridSearchResults"));
            if (Directory.Exists(output))
                Directory.Delete(output, true);
            Directory.CreateDirectory(output);

            WriteReference(Path.Combine(output, "reference.csv"), references);
            var candidates = BuildCandidates();
            var results = new List<Metrics>(candidates.Count);

            foreach (var candidate in candidates)
                results.Add(Evaluate(candidate, chains, references, output));

            WriteSummary(Path.Combine(output, "summary.csv"), results);
            WriteMetadata(Path.Combine(output, "metadata.json"), chains[0].PhysBone, candidates.Count);
            WriteBest(Path.Combine(output, "best-candidate.txt"), results);

            var peakReferenceMotion = results.Max(result => result.PeakReferenceDeflection);

            yield return new ExitPlayMode();

            Time.captureFramerate = oldCaptureFramerate;
            Time.fixedDeltaTime = oldFixedDeltaTime;
            Time.timeScale = oldTimeScale;
            // The fixture lives only in Play Mode and is removed when Play Mode exits.

            Assert.That(candidates.Count, Is.EqualTo(64));
            Assert.That(results.Count, Is.EqualTo(64));
            Assert.That(references.All(samples => samples.Count == FrameCount + 1), Is.True);
            Assert.That(peakReferenceMotion, Is.GreaterThan(0.0001f),
                "PhysBone did not move in Play Mode. The reference capture is unusable.");
            Assert.That(File.Exists(Path.Combine(output, "reference.csv")), Is.True);
            Assert.That(File.Exists(Path.Combine(output, "summary.csv")), Is.True);
            Assert.That(Directory.GetFiles(output, "candidate-*.csv").Length, Is.EqualTo(64));
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

            // Four radial skirt panels, each represented by two consecutive spring segments.
            for (var i = 0; i < 4; i++)
            {
                var radial = Quaternion.Euler(0.0f, i * 90.0f, 0.0f) * Vector3.forward;
                var root = new GameObject($"SkirtRoot_{i}");
                root.transform.SetParent(hips, false);
                root.transform.localPosition = radial * 0.08f;

                var axis = (radial * 0.12f + Vector3.down).normalized;
                var mid = new GameObject($"SkirtMid_{i}");
                mid.transform.SetParent(root.transform, false);
                mid.transform.localPosition = axis * 0.18f;

                var tip = new GameObject($"SkirtTip_{i}");
                tip.transform.SetParent(mid.transform, false);
                tip.transform.localPosition = axis * 0.18f;

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
                    Nodes = new[] { root.transform, mid.transform, tip.transform },
                    PhysBone = physBone,
                    BoneAxes = new[]
                    {
                        mid.transform.localPosition.normalized,
                        tip.transform.localPosition.normalized,
                    },
                    BoneLengths = new[]
                    {
                        mid.transform.localPosition.magnitude,
                        tip.transform.localPosition.magnitude,
                    },
                    InitialLocalRotations = new[]
                    {
                        root.transform.localRotation,
                        mid.transform.localRotation,
                    },
                });
            }

            return avatar;
        }

        private static Vector3 BodyPosition(float time)
        {
            const float distance = 0.14f;
            const float outbound = 0.25f;
            const float returning = 0.20f;

            if (time < outbound)
                return Vector3.right * (distance * time / outbound);
            if (time < outbound + returning)
                return Vector3.right * (distance * (1.0f - (time - outbound) / returning));
            return Vector3.zero;
        }

        private static Quaternion BodyRotation(float time)
        {
            if (time >= ImpulseEndTime)
                return Quaternion.identity;
            return Quaternion.Euler(0.0f, 0.0f,
                Mathf.Sin(Mathf.PI * time / ImpulseEndTime) * 8.0f);
        }

        private static Sample Capture(Chain chain, Transform hips, float time)
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
            IReadOnlyList<Chain> chains,
            IReadOnlyList<Sample>[] references,
            string outputDirectory)
        {
            var metrics = new Metrics { Candidate = candidate, SettlingTime = -1.0f };
            var csv = new StringBuilder();
            csv.AppendLine("frame,time,chain,segment,referenceX,referenceY,referenceZ,candidateX,candidateY,candidateZ,errorMeters");

            double squaredError = 0.0;
            var sampleCount = 0;
            var maxError = 0.0f;
            var peakReference = 0.0f;
            var peakCandidate = 0.0f;
            var candidateTracks = new List<Vector3>[chains.Count][];

            for (var chainIndex = 0; chainIndex < chains.Count; chainIndex++)
            {
                var chain = chains[chainIndex];
                var samples = references[chainIndex];
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
                {
                    tracks[segment] = new List<Vector3>(samples.Count) { currentTails[segment] };
                }

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

                        csv.Append(frame.ToString(CultureInfo.InvariantCulture)).Append(',')
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

                candidateTracks[chainIndex] = tracks;
            }

            metrics.RmsError = sampleCount == 0 ? 0.0f : Mathf.Sqrt((float)(squaredError / sampleCount));
            metrics.MaxError = maxError;
            metrics.PeakReferenceDeflection = peakReference;
            metrics.PeakCandidateDeflection = peakCandidate;
            metrics.SettlingTime = FindSettlingTime(references, candidateTracks);
            File.WriteAllText(Path.Combine(outputDirectory, $"candidate-{candidate.Id:D3}.csv"),
                csv.ToString(), new UTF8Encoding(false));
            return metrics;
        }

        private static float FindSettlingTime(
            IReadOnlyList<Sample>[] references,
            List<Vector3>[][] tracks)
        {
            var first = Mathf.CeilToInt(ImpulseEndTime * SampleRate);
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
                    return frame / (float)SampleRate - ImpulseEndTime;
            }
            return -1.0f;
        }

        private static void WriteReference(string path, IReadOnlyList<Sample>[] references)
        {
            var csv = new StringBuilder();
            csv.AppendLine("frame,time,chain,segment,anchorX,anchorY,anchorZ,parentQX,parentQY,parentQZ,parentQW,tailX,tailY,tailZ");
            for (var chain = 0; chain < references.Length; chain++)
            for (var frame = 0; frame < references[chain].Count; frame++)
            for (var segment = 0; segment < references[chain][frame].ReferenceTails.Length; segment++)
            {
                var sample = references[chain][frame];
                var anchor = sample.Anchors[segment];
                var parentRotation = sample.ParentRotations[segment];
                var tail = sample.ReferenceTails[segment];

                csv.Append(frame.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(F(sample.Time)).Append(',').Append(chain.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(segment.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(F(anchor.x)).Append(',').Append(F(anchor.y)).Append(',')
                    .Append(F(anchor.z)).Append(',').Append(F(parentRotation.x)).Append(',')
                    .Append(F(parentRotation.y)).Append(',').Append(F(parentRotation.z)).Append(',')
                    .Append(F(parentRotation.w)).Append(',').Append(F(tail.x)).Append(',')
                    .Append(F(tail.y)).Append(',').Append(F(tail.z)).AppendLine();
            }
            File.WriteAllText(path, csv.ToString(), new UTF8Encoding(false));
        }

        private static void WriteSummary(string path, IReadOnlyList<Metrics> results)
        {
            var csv = new StringBuilder();
            csv.AppendLine("candidateId,stiffnessScale,inertiaScale,gravityScale,rmsErrorMeters,maxErrorMeters,peakReferenceDeflectionMeters,peakCandidateDeflectionMeters,settlingTimeSeconds");
            foreach (var item in results)
            {
                csv.Append(item.Candidate.Id.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(F(item.Candidate.Scales.Stiffness)).Append(',').Append(F(item.Candidate.Scales.Inertia)).Append(',')
                    .Append(F(item.Candidate.Scales.Gravity)).Append(',').Append(F(item.RmsError)).Append(',')
                    .Append(F(item.MaxError)).Append(',').Append(F(item.PeakReferenceDeflection)).Append(',')
                    .Append(F(item.PeakCandidateDeflection)).Append(',').Append(F(item.SettlingTime)).AppendLine();
            }
            File.WriteAllText(path, csv.ToString(), new UTF8Encoding(false));
        }

        private static void WriteMetadata(string path, VRCPhysBone physBone, int candidateCount)
        {
            var json = new StringBuilder();
            json.AppendLine("{");
            json.AppendLine("  \"scenario\": \"lateral-impulse-and-settle\",");
            json.Append("  \"unityVersion\": \"").Append(Application.unityVersion).AppendLine("\",");
            json.Append("  \"sampleRateHz\": ").Append(SampleRate.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"frameCount\": ").Append(FrameCount.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"chainCount\": 4,").AppendLine();
            json.Append("  \"segmentCountPerChain\": ").Append(SegmentCountPerChain.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"simulatedJointPairCount\": ")
                .Append((4 * SegmentCountPerChain).ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.Append("  \"candidateCount\": ").Append(candidateCount.ToString(CultureInfo.InvariantCulture)).AppendLine(",");
            json.AppendLine("  \"physBone\": {");
            json.AppendLine("    \"version\": \"1.0\",");
            json.AppendLine("    \"integrationType\": \"Simplified\",");
            json.Append("    \"pull\": ").Append(F(physBone.pull)).AppendLine(",");
            json.Append("    \"spring\": ").Append(F(physBone.spring)).AppendLine(",");
            json.Append("    \"gravity\": ").Append(F(physBone.gravity)).AppendLine(",");
            json.Append("    \"gravityFalloff\": ").Append(F(physBone.gravityFalloff)).AppendLine();
            json.AppendLine("  },");
            json.AppendLine("  \"scaleAxes\": {");
            json.AppendLine("    \"stiffness\": [4, 8, 12, 16],");
            json.AppendLine("    \"inertia\": [1, 2, 3, 4],");
            json.AppendLine("    \"gravity\": [2, 5, 10, 20]");
            json.AppendLine("  },");
            json.AppendLine("  \"referenceAlgorithm\": \"https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_springBone-1.0/README.md#update-procedure\",");
            json.AppendLine("  \"settlingCriterion\": \"Candidate tail remains within 0.005m of the captured VRC terminal tail pose for all remaining samples.\",");
            json.AppendLine("  \"notes\": \"Synthetic four-panel skirt fixture; two consecutive measured segments per PhysBone chain. Candidate motion applies each updated head-node rotation before evaluating the next VRMC_springBone 1.0 joint pair. The fixture uses unit scale and world space. No colliders, limits, or additional body-motion scenarios are included.\",");
            json.AppendLine("  \"artifacts\": [\"reference.csv\", \"candidate-001.csv through candidate-064.csv\", \"summary.csv\"]");
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
                ? $"Best settled candidate by RMS position error: #{best.Candidate.Id:D3}"
                : $"No candidate met the terminal-pose tolerance; best RMS candidate: #{best.Candidate.Id:D3}");
            text.AppendLine($"Stiffness scale: {F(best.Candidate.Scales.Stiffness)}");
            text.AppendLine($"Inertia scale: {F(best.Candidate.Scales.Inertia)}");
            text.AppendLine($"Gravity scale: {F(best.Candidate.Scales.Gravity)}");
            text.AppendLine($"RMS error: {F(best.RmsError)} m");
            text.AppendLine($"Maximum error: {F(best.MaxError)} m");
            text.AppendLine($"Settling time: {F(best.SettlingTime)} s (-1 means not settled)");
            text.AppendLine($"Candidates meeting settling tolerance: {settled.Count}/{results.Count}");
            File.WriteAllText(path, text.ToString(), new UTF8Encoding(false));
        }

        private static string F(float value)
        {
            return value.ToString("0.######", CultureInfo.InvariantCulture);
        }
    }
}
