using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
	public class PathI : Path<int2>
	{
		public PathI() : base() { }
		public PathI(int capacity = 0) : base(capacity) { }
		public PathI(IReadOnlyCollection<int2> path) : base(path) { }
	}

	public class PathF : Path<float2>
	{
		public PathF() : base() { }
		public PathF(int capacity = 0) : base(capacity) { }
		public PathF(IReadOnlyCollection<float2> path) : base(path) { }
	}

	public class Path<T> : IReadOnlyCollection<T>
		where T : unmanaged
	{
		private NativeList<T> list;

		public int Count => list.Length;

		public T this[int index]
		{
			get => list[index];
			set => list[index] = value;
		}

		public Path() => list = new NativeList<T>(Allocator.Temp);
		public Path(int capacity = 0) => list = new NativeList<T>(capacity, Allocator.Temp);
		public Path(IReadOnlyCollection<T> path)
		{
			list = new NativeList<T>(path.Count, Allocator.Temp);
			foreach (var pt in path)
				list.Add(pt);
		}

		public void Add(T pt) => list.Add(pt);
		public void AddRange(NativeArray<float2> seq, float scale) => ((Path<int2>)(object)this).list.AddRange(seq, scale);
		public void AddRange(NativeArray<int2> seq, float scale) => ((Path<float2>)(object)this).list.AddRange(seq, scale);
		public void RemoveAt(int index) => list.RemoveAt(index);
		public void Clear() => list.Clear();
		public void EnsureCapacity(int capacity) => list.EnsureCapacity(capacity);
		public void Resize(int size, NativeArrayOptions options) => list.Resize(size, options);
		public void Reverse() => list.AsArray().Reverse();
		public NativeArray<T> AsArray() => list.AsArray();

		IEnumerator<T> IEnumerable<T>.GetEnumerator() => list.GetEnumerator();
		IEnumerator IEnumerable.GetEnumerator() => list.GetEnumerator();

		public override string ToString()
			=> string.Join(", ", this);

		public static implicit operator NativeList<T>(Path<T> path) => path.list;
		public static implicit operator NativeArray<T>(Path<T> path) => path.list.AsArray();
	}

	public class PathsI : Paths<int2>
	{
		public PathsI() : base() { }
		public PathsI(int capacity = 0) : base(capacity) { }
		public PathsI(IEnumerable<PathI> paths) : base(paths) { }
	}

	public class PathsF : Paths<float2>
	{
		public PathsF() : base() { }
		public PathsF(int capacity = 0) : base(capacity) { }
		public PathsF(IEnumerable<PathF> paths) : base(paths) { }
	}

	public class Paths<T> : IEnumerable<NativeArray<T>>
		where T : unmanaged
	{
		private NativeSlicedList<T> list;

		public int Count => list.Count;
		public int ItemsCount => list.ItemsCount;

		public NativeArray<T> this[int index] => list[index];

		public Paths() => list = new NativeSlicedList<T>(Allocator.Temp);
		public Paths(int capacity = 0) => list = new NativeSlicedList<T>(capacity, Allocator.Temp);
		public Paths(IEnumerable<Path<T>> paths) => list = new NativeSlicedList<T>(paths, Allocator.Temp);

		public void EnsureCapacity(int itemsCount) => list.EnsureItemsCapacity(itemsCount);
		public void Add(Path<T> path) => list.Add((NativeArray<T>)path);
		public void Add(NativeArray<T> slice) => list.Add(slice);
		public void Add(NativeArray<float2> slice, float scale) => ((Paths<int2>)(object)this).list.Add(slice, scale);
		public void Add(NativeArray<int2> slice, float scale) => ((Paths<float2>)(object)this).list.Add(slice, scale);
		public void Clear() => list.Clear();

		IEnumerator<NativeArray<T>> IEnumerable<NativeArray<T>>.GetEnumerator() => list.GetEnumerator();
		IEnumerator IEnumerable.GetEnumerator() => list.GetEnumerator();

		public override string ToString()
			=> string.Join(Environment.NewLine, this);

		public static implicit operator NativeSlicedList<T>(Paths<T> paths) => paths.list;
	}
}