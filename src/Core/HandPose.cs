using System;

namespace Kinect2VMC.Core
{
    /// <summary>
    /// What the hand is doing, as far as Kinect v2 actually allows.
    ///
    /// Kinect gives five joints per hand -- Wrist, Hand, HandTip and Thumb -- and no per-finger
    /// joint angles. So individual finger poses are out of reach, but the SHAPE of the hand is
    /// not: a fist brings the fingertips in towards the palm and towards the thumb, while an open
    /// hand spreads them away from both. Those two distances are what this measures.
    ///
    /// Both values are normalised by the hand's own size, so they do not change when the user
    /// moves nearer to or further from the sensor.
    /// </summary>
    public struct HandMetrics
    {
        /// <summary>Distance from the fingertip joint to the thumb tip, in METRES.</summary>
        public float TipToThumb;

        /// <summary>Distance from the wrist to the fingertip, in METRES.</summary>
        public float TipToWrist;

        /// <summary>
        /// Straight-line distance from wrist to fingertip divided by the wrist-to-hand bone
        /// length. An open hand reaches further than a closed one, which folds back on itself.
        /// </summary>
        public float Reach;

        /// <summary>True when all of the joints needed were present in the frame.</summary>
        public bool Valid;

        /// <summary>
        /// 0 = closed fist, 1 = open hand. Derived from <see cref="TipToThumb"/> against a
        /// calibrated range.
        /// </summary>
        public float Openness;

        public HandPose Pose;
    }

    /// <summary>Coarse hand shapes, ordered from closed to open.</summary>
    public enum HandPose
    {
        Unknown = 0,
        Fist,
        LooseFist,
        Open
    }

    /// <summary>
    /// Turns Kinect's five hand joints into a coarse hand pose.
    ///
    /// The measurement is the RAW fingertip-to-thumb distance in metres. An earlier version
    /// divided it by the wrist-to-"hand" bone, which is only a few centimetres long, so the ratios
    /// came out around 1.0-3.5 and the fixed 0.35/0.75 thresholds never fired -- every sample
    /// classified as "open" no matter what the hand did. Absolute distances avoid that entirely
    /// and matched a real recording: on a hand the sensor could see, the distance ran from about
    /// 0.5 cm closed to 14 cm open.
    ///
    /// The thresholds are per-hand configurable because Kinect tracks the two hands quite
    /// differently: one user's recording produced a clearly bimodal distribution on one hand and
    /// an almost constant value on the other.
    /// </summary>
    public static class HandPoseEstimator
    {
        /// <summary>Fingertip-to-thumb distance in metres at or below which the hand is a fist.</summary>
        public static float FistDistance = 0.04f;

        /// <summary>Fingertip-to-thumb distance in metres at or above which the hand is open.</summary>
        public static float OpenDistance = 0.10f;

        /// <summary>
        /// How a new hand reading is combined with the previous one.
        ///
        /// The measured states overlap in practice -- sensor noise, plus the fact that a real hand
        /// passes through every intermediate shape -- so the raw classification flickers. Driving a
        /// model that has only two visual states from a flickering signal produces jitter, which is
        /// worse than no animation at all. Hysteresis fixes that by refusing to leave the current
        /// state until the reading clearly belongs to the other one.
        /// </summary>
        public enum HandStabilizer
        {
            /// <summary>Report each frame's raw classification. Diagnostic only; it flickers.</summary>
            None,

            /// <summary>Latch to a state, switching only when the reading clearly crosses over.</summary>
            Hysteresis
        }

        /// <summary>Distance in metres a fist must exceed before it is allowed to become open.</summary>
        public static float OpenReleaseDistance = 0.07f;

        /// <summary>Distance in metres an open hand must fall below before it becomes a fist.</summary>
        public static float FistEngageDistance = 0.045f;

        /// <summary>
        /// How long a new reading must persist before the state is allowed to change, in seconds.
        ///
        /// Distance thresholds alone are not enough. The hand joints are inferred and jitter, so a
        /// reading hovering near a threshold flips the state every frame, and a model driven from
        /// that twitches continuously. Requiring the candidate to be consistent for a short while
        /// costs a small delay on a deliberate gesture and removes the flicker entirely.
        /// </summary>
        public static float SwitchHoldSeconds = 0.15f;

        /// <summary>
        /// Time-based debounce. Call once per frame, passing the frame's time and the raw
        /// classification; it returns the state that should actually be shown.
        /// </summary>
        public static HandPose StabilizeOverTime(float tipToThumb, HandPose previous, float time,
            ref HandPose candidate, ref float candidateSince, HandStabilizer mode)
        {
            HandPose raw = Stabilize(tipToThumb, previous, mode);

            // No hold requested: commit at once, which is what a single-frame test needs.
            if (SwitchHoldSeconds <= 0f) { return raw; }

            if (raw == previous)
            {
                // Already showing this: nothing is pending.
                candidate = previous;
                candidateSince = time;
                return previous;
            }

            if (raw != candidate)
            {
                // A different state is now being proposed; restart the hold.
                candidate = raw;
                candidateSince = time;
                return previous;
            }

            // The same change has been proposed continuously for long enough: commit it.
            if (time - candidateSince >= SwitchHoldSeconds) { return raw; }
            return previous;
        }

        /// <summary>
        /// Collapses a raw reading into a stable two-state result.
        ///
        /// <paramref name="previous"/> is the latched state, and it only changes when the reading
        /// is unambiguous: a fist is declared only below <see cref="FistEngageDistance"/> and an
        /// open hand only above <see cref="OpenReleaseDistance"/>. Anything in between holds
        /// whatever was already showing, which is exactly where a half-closed hand lives.
        /// </summary>
        public static HandPose Stabilize(float tipToThumb, HandPose previous, HandStabilizer mode)
        {
            if (mode == HandStabilizer.None)
            {
                if (tipToThumb <= FistDistance) { return HandPose.Fist; }
                if (tipToThumb >= OpenDistance) { return HandPose.Open; }
                return HandPose.LooseFist;
            }

            if (previous == HandPose.Fist)
            {
                return tipToThumb >= OpenReleaseDistance ? HandPose.Open : HandPose.Fist;
            }

            // From open or unknown, latch to a fist only once the hand clearly closes.
            return tipToThumb <= FistEngageDistance ? HandPose.Fist : HandPose.Open;
        }

        /// <summary>
        /// Measures one hand from its four joints, in character space.
        ///
        /// <paramref name="fingerJointsTracked"/> gates the whole estimate. Kinect reports the
        /// fingertip and thumb joints as inferred rather than tracked, and when it loses them it
        /// leaves their coordinates at zero. Without this gate a dropped hand reads as a fingertip
        /// sitting exactly on the thumb -- a confident, entirely fabricated fist. The user's own
        /// recordings contained exactly those readings.
        /// </summary>
        public static HandMetrics Compute(Vec3 wrist, Vec3 hand, Vec3 handTip, Vec3 thumb,
            bool fingerJointsTracked)
        {
            var metrics = new HandMetrics();

            if (!fingerJointsTracked) { return metrics; }

            if (!wrist.IsFinite() || !hand.IsFinite() || !handTip.IsFinite() || !thumb.IsFinite())
            {
                return metrics;
            }

            float palm = (hand - wrist).Length;
            if (palm < 1e-4f) { return metrics; }

            // Two joints reported at the same place is a dropout, not a fist.
            if ((handTip - thumb).Length < 1e-4f) { return metrics; }

            // Raw metres. See the class comment for why this is not normalised by the palm bone.
            metrics.TipToThumb = (handTip - thumb).Length;
            metrics.TipToWrist = (handTip - wrist).Length;
            metrics.Reach = metrics.TipToWrist / palm;
            metrics.Valid = true;

            float range = OpenDistance - FistDistance;
            float openness = range > 1e-4f ? (metrics.TipToThumb - FistDistance) / range : 0f;
            if (openness < 0f) { openness = 0f; }
            if (openness > 1f) { openness = 1f; }
            metrics.Openness = openness;

            if (metrics.TipToThumb <= FistDistance) { metrics.Pose = HandPose.Fist; }
            else if (metrics.TipToThumb >= OpenDistance) { metrics.Pose = HandPose.Open; }
            else { metrics.Pose = HandPose.LooseFist; }

            return metrics;
        }

        /// <summary>Convenience overload for callers that already know the joints were present.</summary>
        public static HandMetrics Compute(Vec3 wrist, Vec3 hand, Vec3 handTip, Vec3 thumb)
        {
            return Compute(wrist, hand, handTip, thumb, true);
        }
    }
}
