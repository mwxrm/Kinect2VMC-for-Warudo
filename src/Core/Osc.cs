using System;
using System.Collections.Generic;
using System.Text;

namespace Kinect2VMC.Core
{
    /// <summary>
    /// Minimal OSC 1.0 message writer.
    ///
    /// Only what VMC Protocol needs: an address pattern, a comma-prefixed type tag
    /// string, and 4-byte-aligned big-endian arguments. Strings are UTF-8 and
    /// null-terminated; OSC requires the terminator to be inside the padding.
    /// </summary>
    public sealed class OscMessageWriter
    {
        private byte[] _buffer;
        private int _length;
        private readonly List<char> _typeTags = new List<char>(16);
        private bool _addressWritten;
        private int _typeTagOffset;
        private int _typeTagReserved;

        public OscMessageWriter(int initialCapacity = 512)
        {
            if (initialCapacity < 64) { initialCapacity = 64; }
            _buffer = new byte[initialCapacity];
        }

        public int Length { get { return _length; } }
        public byte[] Buffer { get { return _buffer; } }

        /// <summary>Starts a new message. Reuses the internal buffer, so no allocation per frame.</summary>
        public void Begin(string address)
        {
            _length = 0;
            _typeTags.Clear();
            _addressWritten = false;

            WritePaddedString(address);

            // Reserve the type tag string. It is always at least "," + null + padding,
            // and it can never exceed the number of arguments we support. NOTE: this must
            // assign the FIELD, not a local: shadowing it here leaves the reservation at
            // zero, and End() then overwrites the argument payload while laying the tags
            // down, which silently corrupts every float in the packet.
            _typeTagOffset = _length;
            _typeTagReserved = 4;
            EnsureCapacity(_length + _typeTagReserved);
            _length += _typeTagReserved;
            _typeTags.Add(',');
            _addressWritten = true;
        }

        public void AddFloat(float value)
        {
            RequireAddress();
            _typeTags.Add('f');
            EnsureCapacity(_length + 4);
            WriteBigEndianFloat(_buffer, _length, value);
            _length += 4;
        }

        public void AddInt(int value)
        {
            RequireAddress();
            _typeTags.Add('i');
            EnsureCapacity(_length + 4);
            WriteBigEndianInt(_buffer, _length, value);
            _length += 4;
        }

        public void AddString(string value)
        {
            RequireAddress();
            _typeTags.Add('s');
            WritePaddedString(value ?? string.Empty);
        }

        /// <summary>Writes the accumulated type tag string into the reserved slot and returns the packet length.</summary>
        public int End()
        {
            RequireAddress();

            string tags = new string(_typeTags.ToArray());
            int needed = PaddedLength(tags);
            if (needed > _typeTagReserved)
            {
                // Grow the reservation by shifting the argument payload to the right.
                int extra = needed - _typeTagReserved;
                EnsureCapacity(_length + extra);
                Array.Copy(_buffer, _typeTagOffset + _typeTagReserved,
                           _buffer, _typeTagOffset + needed,
                           _length - (_typeTagOffset + _typeTagReserved));
                _length += extra;
                _typeTagReserved = needed;
            }

            // Zero the reserved region, then lay the tags down inside it.
            Array.Clear(_buffer, _typeTagOffset, _typeTagReserved);
            byte[] tagBytes = Encoding.UTF8.GetBytes(tags);
            Array.Copy(tagBytes, 0, _buffer, _typeTagOffset, tagBytes.Length);

            return _length;
        }

        private void RequireAddress()
        {
            if (!_addressWritten)
            {
                throw new InvalidOperationException("Call Begin(address) before adding arguments.");
            }
        }

        private void WritePaddedString(string value)
        {
            byte[] bytes = Encoding.UTF8.GetBytes(value ?? string.Empty);
            int total = PaddedLength(value ?? string.Empty, bytes.Length);
            EnsureCapacity(_length + total);
            Array.Copy(bytes, 0, _buffer, _length, bytes.Length);
            for (int i = bytes.Length; i < total; i++) { _buffer[_length + i] = 0; }
            _length += total;
        }

        private static int PaddedLength(string value)
        {
            return PaddedLength(value, Encoding.UTF8.GetByteCount(value ?? string.Empty));
        }

        private static int PaddedLength(string value, int byteCount)
        {
            // At least one null terminator, then round up to a multiple of four.
            int withTerminator = byteCount + 1;
            int remainder = withTerminator % 4;
            return remainder == 0 ? withTerminator : withTerminator + (4 - remainder);
        }

        private void EnsureCapacity(int required)
        {
            if (_buffer.Length >= required) { return; }
            int size = _buffer.Length;
            while (size < required) { size *= 2; }
            Array.Resize(ref _buffer, size);
        }

        private static void WriteBigEndianFloat(byte[] buffer, int offset, float value)
        {
            byte[] raw = BitConverter.GetBytes(value);
            if (BitConverter.IsLittleEndian) { Array.Reverse(raw); }
            buffer[offset] = raw[0];
            buffer[offset + 1] = raw[1];
            buffer[offset + 2] = raw[2];
            buffer[offset + 3] = raw[3];
        }

        private static void WriteBigEndianInt(byte[] buffer, int offset, int value)
        {
            buffer[offset] = (byte)(value >> 24);
            buffer[offset + 1] = (byte)(value >> 16);
            buffer[offset + 2] = (byte)(value >> 8);
            buffer[offset + 3] = (byte)value;
        }
    }

    /// <summary>A decoded OSC argument, used by the monitor and by round-trip tests.</summary>
    public struct OscArgument
    {
        public char Type;
        public float FloatValue;
        public int IntValue;
        public string StringValue;

        public override string ToString()
        {
            switch (Type)
            {
                case 'f': return FloatValue.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
                case 'i': return IntValue.ToString(System.Globalization.CultureInfo.InvariantCulture);
                case 's': return "\"" + StringValue + "\"";
                default: return Type.ToString();
            }
        }
    }

    public sealed class OscMessage
    {
        public string Address;
        public OscArgument[] Arguments = new OscArgument[0];

        public override string ToString()
        {
            var sb = new StringBuilder(Address);
            for (int i = 0; i < Arguments.Length; i++)
            {
                sb.Append(' ').Append(Arguments[i]);
            }
            return sb.ToString();
        }
    }

    /// <summary>
    /// Minimal OSC 1.0 message reader. Deliberately strict: returns false instead of
    /// throwing, so the monitor can keep running on malformed traffic.
    /// </summary>
    public static class OscMessageReader
    {
        public static bool TryParse(byte[] buffer, int length, out OscMessage message)
        {
            message = null;
            if (buffer == null || length < 8) { return false; }

            // A bundle starts with "#bundle"; VMC senders are free to bundle, so unwrap
            // the first element only if it is a single-element bundle.
            if (length >= 8 && buffer[0] == (byte)'#' && buffer[1] == (byte)'b')
            {
                return false;
            }

            int offset = 0;
            string address;
            if (!TryReadPaddedString(buffer, length, ref offset, out address)) { return false; }
            if (address.Length == 0 || address[0] != '/') { return false; }

            string tags;
            if (!TryReadPaddedString(buffer, length, ref offset, out tags)) { return false; }
            if (tags.Length == 0 || tags[0] != ',') { return false; }

            var args = new List<OscArgument>(tags.Length);
            for (int i = 1; i < tags.Length; i++)
            {
                char t = tags[i];
                var arg = new OscArgument { Type = t };
                switch (t)
                {
                    case 'f':
                        if (offset + 4 > length) { return false; }
                        arg.FloatValue = ReadBigEndianFloat(buffer, offset);
                        offset += 4;
                        break;
                    case 'i':
                        if (offset + 4 > length) { return false; }
                        arg.IntValue = ReadBigEndianInt(buffer, offset);
                        offset += 4;
                        break;
                    case 's':
                        string s;
                        if (!TryReadPaddedString(buffer, length, ref offset, out s)) { return false; }
                        arg.StringValue = s;
                        break;
                    case 'T': arg.IntValue = 1; break;
                    case 'F': arg.IntValue = 0; break;
                    default:
                        // Unknown type: the packet cannot be decoded reliably.
                        return false;
                }
                args.Add(arg);
            }

            message = new OscMessage { Address = address, Arguments = args.ToArray() };
            return true;
        }

        private static bool TryReadPaddedString(byte[] buffer, int length, ref int offset, out string value)
        {
            value = null;
            int start = offset;
            int end = start;
            while (end < length && buffer[end] != 0) { end++; }
            if (end >= length) { return false; }

            value = Encoding.UTF8.GetString(buffer, start, end - start);

            int consumed = end - start + 1;
            int padded = consumed + ((4 - (consumed % 4)) % 4);
            if (start + padded > length) { return false; }
            offset = start + padded;
            return true;
        }

        private static float ReadBigEndianFloat(byte[] buffer, int offset)
        {
            byte[] raw = new byte[4];
            raw[0] = buffer[offset];
            raw[1] = buffer[offset + 1];
            raw[2] = buffer[offset + 2];
            raw[3] = buffer[offset + 3];
            if (BitConverter.IsLittleEndian) { Array.Reverse(raw); }
            return BitConverter.ToSingle(raw, 0);
        }

        private static int ReadBigEndianInt(byte[] buffer, int offset)
        {
            return (buffer[offset] << 24) | (buffer[offset + 1] << 16) | (buffer[offset + 2] << 8) | buffer[offset + 3];
        }
    }
}
