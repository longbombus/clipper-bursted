using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
	public class PathI : List<int2>
	{
		public PathI() : base() { }
		public PathI(int capacity = 0) : base(capacity) { }
		public PathI(IEnumerable<int2> path) : base(path) { }
		public override string ToString()
		{
			return string.Join(", ", this);
		}
	}

	public class PathsI : List<PathI>
	{
		public PathsI() : base() { }
		public PathsI(int capacity = 0) : base(capacity) { }
		public PathsI(IEnumerable<PathI> paths) : base(paths) { }
		public override string ToString()
		{
			return string.Join(Environment.NewLine, this);
		}
	}

	public class PathF : IReadOnlyCollection<float2>
	{
		private NativeList<float2> list;

		public int Count => list.Length;

		public float2 this[int index]
		{
			get => list[index];
			set => list[index] = value;
		}

		public PathF()
		{
			list = new NativeList<float2>(Allocator.Temp);
		}

		public PathF(int capacity = 0)
		{
			list = new NativeList<float2>(capacity, Allocator.Temp);
		}

		public PathF(IReadOnlyCollection<float2> path)
		{
			list = new NativeList<float2>(path.Count, Allocator.Temp);
			foreach (var pt in path)
				list.Add(pt);
		}

		public void Add(float2 pt) => list.Add(pt);
		public void RemoveAt(int index) => list.RemoveAt(index);
		public void Clear() => list.Clear();
		public void EnsureCapacity(int capacity) => list.Capacity = math.max(list.Capacity, capacity);
		public void Reverse() => list.AsArray().Reverse();

		IEnumerator<float2> IEnumerable<float2>.GetEnumerator() => list.GetEnumerator();
		IEnumerator IEnumerable.GetEnumerator() => list.GetEnumerator();

		public override string ToString()
			=> string.Join(", ", this);
	}

	public class PathsF : List<PathF>
	{
		public PathsF() : base() { }
		public PathsF(int capacity = 0) : base(capacity) { }
		public PathsF(IEnumerable<PathF> paths) : base(paths) { }
		public override string ToString()
			=> string.Join(Environment.NewLine, this);
	}
}