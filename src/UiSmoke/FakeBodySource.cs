using System;
using System.Collections.Generic;
using Kinect2VMC.App;
using Kinect2VMC.Core;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// A scripted stand-in for <see cref="KinectBodySource"/>.
    ///
    /// It serves a queue of synthetic frames, so the whole pipeline can be driven
    /// deterministically: sensor loss, body loss, calibration, mirroring, root translation and
    /// the VMC encoding are all exercised without a device.
    /// </summary>
    internal sealed class FakeBodySource : IBodySource
    {
        private readonly Queue<SkeletonFrameData> _script = new Queue<SkeletonFrameData>();

        /// <summary>Frame served when the script runs dry.</summary>
        public SkeletonFrameData HoldFrame;

        public bool Available = true;
        public string IdText = "FAKE-SENSOR-0001";
        public float Rate = 30f;
        public int Bodies = 1;
        public string Status = "已连接";

        /// <summary>Device-layer error text; a test can set it to exercise the UI error path.</summary>
        public string Error;
        public bool Disposed;

        /// <summary>How many times EnsureOpen was called, to prove the pipeline keeps it alive.</summary>
        public int OpenCalls;

        /// <summary>Set to serve a body-less frame even though the sensor is available.</summary>
        public bool BodyTracked = true;

        private bool _lastFrameHadBody;

        public void Enqueue(SkeletonFrameData frame)
        {
            _script.Enqueue(frame);
        }

        public void EnqueueRepeated(SkeletonFrameData frame, int count)
        {
            for (int i = 0; i < count; i++) { _script.Enqueue(frame); }
        }

        public bool IsSensorAvailable { get { return Available; } }
        public string SensorId { get { return IdText; } }
        public int TrackedBodyCount { get { return BodyTracked ? Bodies : 0; } }
        public float FrameRate { get { return Rate; } }
        public string StatusText { get { return Status; } }
        public string LastError { get { return Error; } }

        public void EnsureOpen()
        {
            OpenCalls++;
        }

        public bool TryGetFrame(SkeletonFrameData destination)
        {
            // BodyTracked is the authoritative switch: it lets a test declare "the sensor sees
            // nobody" without having to hand-build an untracked frame.
            if (!BodyTracked)
            {
                _lastFrameHadBody = false;
                destination.Clear();
                return false;
            }

            SkeletonFrameData frame;
            if (_script.Count > 0)
            {
                frame = _script.Dequeue();
            }
            else
            {
                frame = HoldFrame;
            }

            if (frame == null)
            {
                _lastFrameHadBody = false;
                destination.Clear();
                return false;
            }

            Copy(frame, destination);
            destination.BodyTracked = true;
            _lastFrameHadBody = true;
            return true;
        }

        /// <summary>
        /// True when the frame most recently copied out carried a body.
        ///
        /// The real device layer exposes this as <c>IsBodyTracked</c> and it is genuinely a
        /// cached value: it only updates when a frame arrives. Modelling that here is what lets
        /// the pipeline test prove it does not depend on the cache.
        /// </summary>
        public bool LastFrameHadBody { get { return _lastFrameHadBody; } }

        public void Dispose()
        {
            Disposed = true;
        }

        public static void Copy(SkeletonFrameData source, SkeletonFrameData destination)
        {
            destination.BodyTracked = source.BodyTracked;
            destination.TrackingId = source.TrackingId;
            destination.Time = source.Time;

            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                destination.Positions[i] = source.Positions[i];
                destination.Tracked[i] = source.Tracked[i];
            }
        }
    }
}
