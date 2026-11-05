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

        private static readonly float[] StiffnessScales = { 1.0f, 2.0f, 4.0f, 8.0f };
        private static readonly float[] InertiaScales = { 0.25f, 0.5f, 0.75f, 1.0f };
        private static readonly float[] GravityScales = { 2.0f, 5.0f, 10.0f, 20.0f };

        private sealed class Chain
        {
            public Transform Root;
            public Transform Mid;
            public VRCPhysBone PhysBone;
            public Vector3 BoneAxis;
            public float BoneLength;
            public Quaternion InitialLocalRotation;
        }

        private sealed class Sample
        {
            public float Time;
            public Vector3 Anchor;
            public Quaternion ParentRotation;
            public Vector3 ReferenceTail;
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

            // Four radial skirt panels, each represented by one measured spring segment.
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
                    Root = root.transform,
                    Mid = mid.transform,
                    PhysBone = physBone,
                    BoneAxis = mid.transform.localPosition.normalized,
                    BoneLength = mid.transform.localPosition.magnitude,
                    InitialLocalRotation = root.transform.localRotation,
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
            return new Sample
            {
                Time = time,
                Anchor = chain.Root.position,
                ParentRotation = hips.rotation,
                ReferenceTail = chain.Mid.position,
            };
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
            csv.AppendLine("frame,time,chain,referenceX,referenceY,referenceZ,candidateX,candidateY,candidateZ,errorMeters");

            double squaredError = 0.0;
            var sampleCount = 0;
            var maxError = 0.0f;
            var peakReference = 0.0f;
            var peakCandidate = 0.0f;
            var candidateTracks = new List<Vector3>[chains.Count];

            for (var chainIndex = 0; chainIndex < chains.Count; chainIndex++)
            {
                var chain = chains[chainIndex];
                var samples = references[chainIndex];
                var physBone = chain.PhysBone;
                var force = VrmRootExporter.ConvertSpringBoneForces(
                    VRCPhysBoneBase.Version.Version_1_0,
                    VRCPhysBoneBase.IntegrationType.Simplified,
                    physBone.pull, physBone.spring, physBone.stiffness,
                    physBone.gravity, physBone.gravityFalloff, candidate.Scales);

                var gravityDirection = new Vector3(force.GravityDir.X, force.GravityDir.Y, force.GravityDir.Z);
                var currentTail = samples[0].Anchor
                    + samples[0].ParentRotation * chain.InitialLocalRotation * chain.BoneAxis * chain.BoneLength;
                var previousTail = currentTail;
                var track = new List<Vector3>(samples.Count) { currentTail };

                for (var frame = 1; frame < samples.Count; frame++)
                {
                    var sample = samples[frame];
                    var deltaTime = sample.Time - samples[frame - 1].Time;
                    var restDirection = sample.ParentRotation * chain.InitialLocalRotation * chain.BoneAxis;

                    // VRMC_springBone 1.0 reference Verlet update for a single segment.
                    var inertia = (currentTail - previousTail) * (1.0f - force.DragForce);
                    var stiffness = restDirection * (deltaTime * force.Stiffness);
                    var gravity = gravityDirection * (deltaTime * force.GravityPower);
                    var nextTail = currentTail + inertia + stiffness + gravity;

                    var fromAnchor = nextTail - sample.Anchor;
                    if (fromAnchor.sqrMagnitude > Mathf.Epsilon)
                        nextTail = sample.Anchor + fromAnchor.normalized * chain.BoneLength;

                    previousTail = currentTail;
                    currentTail = nextTail;
                    track.Add(currentTail);

                    var error = Vector3.Distance(currentTail, sample.ReferenceTail);
                    squaredError += error * error;
                    sampleCount++;
                    maxError = Mathf.Max(maxError, error);

                    var restTail = sample.Anchor + restDirection * chain.BoneLength;
                    peakReference = Mathf.Max(peakReference, Vector3.Distance(sample.ReferenceTail, restTail));
                    peakCandidate = Mathf.Max(peakCandidate, Vector3.Distance(currentTail, restTail));

                    csv.Append(frame.ToString(CultureInfo.InvariantCulture)).Append(',')
                        .Append(F(sample.Time)).Append(',').Append(chainIndex.ToString(CultureInfo.InvariantCulture)).Append(',')
                        .Append(F(sample.ReferenceTail.x)).Append(',').Append(F(sample.ReferenceTail.y)).Append(',')
                        .Append(F(sample.ReferenceTail.z)).Append(',').Append(F(currentTail.x)).Append(',')
                        .Append(F(currentTail.y)).Append(',').Append(F(currentTail.z)).Append(',').Append(F(error)).AppendLine();
                }

                candidateTracks[chainIndex] = track;
            }

            metrics.RmsError = sampleCount == 0 ? 0.0f : Mathf.Sqrt((float)(squaredError / sampleCount));
            metrics.MaxError = maxError;
            metrics.PeakReferenceDeflection = peakReference;
            metrics.PeakCandidateDeflection = peakCandidate;
            metrics.SettlingTime = FindSettlingTime(chains, references, candidateTracks);
            File.WriteAllText(Path.Combine(outputDirectory, $"candidate-{candidate.Id:D3}.csv"),
                csv.ToString(), new UTF8Encoding(false));
            return metrics;
        }

        private static float FindSettlingTime(
            IReadOnlyList<Chain> chains,
            IReadOnlyList<Sample>[] references,
            IReadOnlyList<Vector3>[] tracks)
        {
            var first = Mathf.CeilToInt(ImpulseEndTime * SampleRate);
            for (var frame = first; frame <= FrameCount; frame++)
            {
                var settled = true;
                for (var chainIndex = 0; chainIndex < chains.Count && settled; chainIndex++)
                {
                    var chain = chains[chainIndex];
                    for (var next = frame; next < references[chainIndex].Count; next++)
                    {
                        var sample = references[chainIndex][next];
                        var restDirection = sample.ParentRotation * chain.InitialLocalRotation * chain.BoneAxis;
                        var restTail = sample.Anchor + restDirection * chain.BoneLength;
                        if (Vector3.Distance(tracks[chainIndex][next], restTail) > SettlingTolerance)
                        {
                            settled = false;
                            break;
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
            csv.AppendLine("frame,time,chain,anchorX,anchorY,anchorZ,parentQX,parentQY,parentQZ,parentQW,tailX,tailY,tailZ");
            for (var chain = 0; chain < references.Length; chain++)
            for (var frame = 0; frame < references[chain].Count; frame++)
            {
                var sample = references[chain][frame];
                csv.Append(frame.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(F(sample.Time)).Append(',').Append(chain.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(F(sample.Anchor.x)).Append(',').Append(F(sample.Anchor.y)).Append(',')
                    .Append(F(sample.Anchor.z)).Append(',').Append(F(sample.ParentRotation.x)).Append(',')
                    .Append(F(sample.ParentRotation.y)).Append(',').Append(F(sample.ParentRotation.z)).Append(',')
                    .Append(F(sample.ParentRotation.w)).Append(',').Append(F(sample.ReferenceTail.x)).Append(',')
                    .Append(F(sample.ReferenceTail.y)).Append(',').Append(F(sample.ReferenceTail.z)).AppendLine();
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
            json.Append("  \"chainCount\": 4,");
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
            json.AppendLine("    \"stiffness\": [1, 2, 4, 8],");
            json.AppendLine("    \"inertia\": [0.25, 0.5, 0.75, 1],");
            json.AppendLine("    \"gravity\": [2, 5, 10, 20]");
            json.AppendLine("  },");
            json.AppendLine("  \"referenceAlgorithm\": \"https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_springBone-1.0/README.md#update-procedure\",");
            json.AppendLine("  \"notes\": \"Synthetic four-panel skirt fixture; one measured segment per PhysBone chain. Candidate motion uses the VRMC_springBone 1.0 reference update equation. No colliders, limits, or multi-joint bending are included in this initial calibration.\",");
            json.AppendLine("  \"artifacts\": [\"reference.csv\", \"candidate-001.csv through candidate-064.csv\", \"summary.csv\"]");
            json.AppendLine("}");
            File.WriteAllText(path, json.ToString(), new UTF8Encoding(false));
        }

        private static void WriteBest(string path, IReadOnlyList<Metrics> results)
        {
            var best = results.OrderBy(item => item.RmsError).First();
            var text = new StringBuilder();
            text.AppendLine($"Best candidate by RMS position error: #{best.Candidate.Id:D3}");
            text.AppendLine($"Stiffness scale: {F(best.Candidate.Scales.Stiffness)}");
            text.AppendLine($"Inertia scale: {F(best.Candidate.Scales.Inertia)}");
            text.AppendLine($"Gravity scale: {F(best.Candidate.Scales.Gravity)}");
            text.AppendLine($"RMS error: {F(best.RmsError)} m");
            text.AppendLine($"Maximum error: {F(best.MaxError)} m");
            text.AppendLine($"Settling time: {F(best.SettlingTime)} s (-1 means not settled)");
            File.WriteAllText(path, text.ToString(), new UTF8Encoding(false));
        }

        private static string F(float value)
        {
            return value.ToString("0.######", CultureInfo.InvariantCulture);
        }
    }
}
