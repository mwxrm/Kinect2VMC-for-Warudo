using System;
using System.Globalization;

namespace Kinect2VMC.Core
{
    /// <summary>
    /// Minimal 3D vector in Unity's coordinate convention: left-handed world axes with
    /// +Y up and +Z forward, but right-handed rotation maths (see <see cref="Vec3.Cross"/>).
    /// Kept dependency-free so the whole math layer can be exercised without a sensor.
    /// </summary>
    public struct Vec3 : IEquatable<Vec3>
    {
        public float X;
        public float Y;
        public float Z;

        public Vec3(float x, float y, float z) { X = x; Y = y; Z = z; }

        public static readonly Vec3 Zero = new Vec3(0f, 0f, 0f);
        public static readonly Vec3 Up = new Vec3(0f, 1f, 0f);
        public static readonly Vec3 Down = new Vec3(0f, -1f, 0f);
        public static readonly Vec3 Right = new Vec3(1f, 0f, 0f);
        public static readonly Vec3 Forward = new Vec3(0f, 0f, 1f);
        public static readonly Vec3 Backward = new Vec3(0f, 0f, -1f);

        public static Vec3 operator +(Vec3 a, Vec3 b) { return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        public static Vec3 operator -(Vec3 a, Vec3 b) { return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        public static Vec3 operator -(Vec3 a) { return new Vec3(-a.X, -a.Y, -a.Z); }
        public static Vec3 operator *(Vec3 a, float s) { return new Vec3(a.X * s, a.Y * s, a.Z * s); }
        public static Vec3 operator *(float s, Vec3 a) { return a * s; }
        public static Vec3 operator /(Vec3 a, float s) { return new Vec3(a.X / s, a.Y / s, a.Z / s); }

        public float Length { get { return (float)Math.Sqrt(X * X + Y * Y + Z * Z); } }
        public float LengthSquared { get { return X * X + Y * Y + Z * Z; } }

        public Vec3 Normalized()
        {
            float len = Length;
            if (len < 1e-9f) { return Zero; }
            return new Vec3(X / len, Y / len, Z / len);
        }

        public static float Dot(Vec3 a, Vec3 b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }

        /// <summary>
        /// Standard right-handed cross product.
        ///
        /// This is deliberately the textbook formula even though the coordinate system is
        /// Unity's left-handed one: Unity's rotation quaternions follow the right-hand
        /// rule, and the frame-building code below relies on the two agreeing. Using a
        /// "left-handed cross product" here silently inverts every bone rotation.
        /// </summary>
        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }

        public static Vec3 Lerp(Vec3 a, Vec3 b, float t)
        {
            if (t < 0f) { t = 0f; } else if (t > 1f) { t = 1f; }
            return new Vec3(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
        }

        public static float Distance(Vec3 a, Vec3 b) { return (a - b).Length; }

        /// <summary>Clamps the vector's magnitude to maxLength, preserving direction.</summary>
        public Vec3 ClampedMagnitude(float maxLength)
        {
            float len = Length;
            if (len <= maxLength || len < 1e-9f) { return this; }
            return this * (maxLength / len);
        }

        public bool IsFinite()
        {
            return !(float.IsNaN(X) || float.IsNaN(Y) || float.IsNaN(Z)
                  || float.IsInfinity(X) || float.IsInfinity(Y) || float.IsInfinity(Z));
        }

        public bool Equals(Vec3 other) { return X == other.X && Y == other.Y && Z == other.Z; }
        public override bool Equals(object obj) { return obj is Vec3 && Equals((Vec3)obj); }
        public override int GetHashCode() { return X.GetHashCode() ^ (Y.GetHashCode() << 2) ^ (Z.GetHashCode() >> 2); }

        public override string ToString()
        {
            return string.Format(CultureInfo.InvariantCulture, "({0:F4}, {1:F4}, {2:F4})", X, Y, Z);
        }
    }

    /// <summary>
    /// Left-handed (Unity-convention) quaternion. (x, y, z) is the vector part, w the scalar part.
    /// </summary>
    public struct Quat : IEquatable<Quat>
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Quat(float x, float y, float z, float w) { X = x; Y = y; Z = z; W = w; }

        public static readonly Quat Identity = new Quat(0f, 0f, 0f, 1f);

        /// <summary>Builds a rotation of <paramref name="radians"/> about a unit <paramref name="axis"/>.</summary>
        public static Quat AxisAngle(Vec3 axis, float radians)
        {
            Vec3 n = axis.Normalized();
            if (n.LengthSquared < 0.5f) { return Identity; }
            float half = radians * 0.5f;
            float s = (float)Math.Sin(half);
            return new Quat(n.X * s, n.Y * s, n.Z * s, (float)Math.Cos(half));
        }

        public float LengthSquared { get { return X * X + Y * Y + Z * Z + W * W; } }

        public Quat Normalized()
        {
            float len = (float)Math.Sqrt(LengthSquared);
            if (len < 1e-9f) { return Identity; }
            return new Quat(X / len, Y / len, Z / len, W / len);
        }

        /// <summary>Conjugate. For a unit quaternion this is the inverse rotation.</summary>
        public Quat Conjugate() { return new Quat(-X, -Y, -Z, W); }

        public static Quat Inverse(Quat q)
        {
            float n = q.LengthSquared;
            if (n < 1e-12f) { return Identity; }
            return new Quat(-q.X / n, -q.Y / n, -q.Z / n, q.W / n);
        }

        /// <summary>
        /// Hamilton product. In Unity's left-handed convention, (a * b) means
        /// "apply b first, then a" when acting on a column vector, so the world
        /// rotation of a child is parent * local.
        /// </summary>
        public static Quat operator *(Quat a, Quat b)
        {
            return new Quat(
                a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
                a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
                a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
        }

        /// <summary>
        /// Rotates a vector by this quaternion. Unity's left-handed world axes with
        /// right-handed rotation maths.
        /// </summary>
        public static Vec3 operator *(Quat q, Vec3 v)
        {
            // v' = v + 2w(qv x v) + 2 qv x (qv x v)
            float qx = q.X, qy = q.Y, qz = q.Z, qw = q.W;

            // t = 2 * (qv x v)
            float tx = 2f * (qy * v.Z - qz * v.Y);
            float ty = 2f * (qz * v.X - qx * v.Z);
            float tz = 2f * (qx * v.Y - qy * v.X);

            // v + w*t + qv x t
            return new Vec3(
                v.X + qw * tx + (qy * tz - qz * ty),
                v.Y + qw * ty + (qz * tx - qx * tz),
                v.Z + qw * tz + (qx * ty - qy * tx));
        }

        public static float Dot(Quat a, Quat b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W; }

        /// <summary>Spherical linear interpolation along the shortest arc.</summary>
        public static Quat Slerp(Quat a, Quat b, float t)
        {
            if (t <= 0f) { return a; }
            if (t >= 1f) { return b; }

            float dot = Dot(a, b);
            if (dot < 0f)
            {
                b = new Quat(-b.X, -b.Y, -b.Z, -b.W);
                dot = -dot;
            }

            if (dot > 0.9995f)
            {
                return new Quat(
                    a.X + (b.X - a.X) * t,
                    a.Y + (b.Y - a.Y) * t,
                    a.Z + (b.Z - a.Z) * t,
                    a.W + (b.W - a.W) * t).Normalized();
            }

            float theta0 = (float)Math.Acos(dot);
            float theta = theta0 * t;
            float sinTheta = (float)Math.Sin(theta);
            float sinTheta0 = (float)Math.Sin(theta0);

            float s0 = (float)Math.Cos(theta) - dot * sinTheta / sinTheta0;
            float s1 = sinTheta / sinTheta0;

            return new Quat(
                a.X * s0 + b.X * s1,
                a.Y * s0 + b.Y * s1,
                a.Z * s0 + b.Z * s1,
                a.W * s0 + b.W * s1).Normalized();
        }

        /// <summary>Euler angles in degrees, Unity ZXY order (x=pitch, y=yaw, z=roll).</summary>
        public Vec3 ToEulerDegrees()
        {
            Quat q = Normalized();

            // Unity's Quaternion.Euler inverse (ZXY intrinsic).
            float sinX = -2f * (q.Y * q.Z - q.W * q.X);
            sinX = sinX < -1f ? -1f : (sinX > 1f ? 1f : sinX);
            float x = (float)Math.Asin(sinX);

            float y, z;
            if (Math.Abs(sinX) > 0.9999f)
            {
                // Gimbal lock: fold roll into yaw.
                y = (float)Math.Atan2(2f * (q.X * q.Z + q.W * q.Y), 1f - 2f * (q.Y * q.Y + q.Z * q.Z));
                z = 0f;
            }
            else
            {
                y = (float)Math.Atan2(2f * (q.X * q.Z + q.W * q.Y), 1f - 2f * (q.X * q.X + q.Y * q.Y));
                z = (float)Math.Atan2(2f * (q.X * q.Y + q.W * q.Z), 1f - 2f * (q.X * q.X + q.Z * q.Z));
            }

            const float rad2deg = 57.29577951308232f;
            return new Vec3(x * rad2deg, y * rad2deg, z * rad2deg);
        }

        public bool IsFinite()
        {
            return !(float.IsNaN(X) || float.IsNaN(Y) || float.IsNaN(Z) || float.IsNaN(W)
                  || float.IsInfinity(X) || float.IsInfinity(Y) || float.IsInfinity(Z) || float.IsInfinity(W));
        }

        public bool Equals(Quat other) { return X == other.X && Y == other.Y && Z == other.Z && W == other.W; }
        public override bool Equals(object obj) { return obj is Quat && Equals((Quat)obj); }

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ (Y.GetHashCode() << 2) ^ (Z.GetHashCode() >> 2) ^ (W.GetHashCode() >> 1);
        }

        public override string ToString()
        {
            return string.Format(CultureInfo.InvariantCulture, "({0:F4}, {1:F4}, {2:F4}, {3:F4})", X, Y, Z, W);
        }
    }
}
