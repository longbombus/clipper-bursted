using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Clipper
{
	[BurstCompile]
	public struct NativeSlicedList<T> : INativeDisposable, IEnumerable<NativeArray<T>>
		where T : unmanaged
	{
		private NativeList<T> list;
		private NativeList<int> slices;

		/// <summary> Number of items in all slices. </summary>
		public int ItemsCount => list.Length;

		/// <summary> Number of slices. </summary>
		public int SlicesCount => slices.Length;

		[Obsolete("Use SlicesCount property instead")]
		public int Count => slices.Length;

		public NativeSlicedList(Allocator allocator)
		{
			list = new NativeList<T>(allocator);
			slices = new NativeList<int>(allocator);
		}

		public NativeSlicedList(int itemsCapacity, Allocator allocator)
		{
			list = new NativeList<T>(itemsCapacity, allocator);
			slices = new NativeList<int>(allocator);
		}

		public NativeSlicedList(IEnumerable<IEnumerable<T>> sourceSlices, Allocator allocator)
			: this(allocator)
		{
			foreach (var slice in sourceSlices)
				Add(slice);
		}

		public NativeSlicedList(IReadOnlyCollection<NativeArray<T>> sourceSlices, Allocator allocator)
			: this(GetItemsCount(sourceSlices), allocator)
		{
			foreach (var slice in sourceSlices)
				Add(slice);
		}

		public NativeSlicedList(IReadOnlyCollection<NativeList<T>> sourceSlices, Allocator allocator)
			: this(GetItemsCount(sourceSlices), allocator)
		{
			foreach (var slice in sourceSlices)
				Add(slice);
		}

		public void Dispose()
		{
			list.Dispose();
			slices.Dispose();
		}

		public JobHandle Dispose(JobHandle inputDeps)
			=> JobHandle.CombineDependencies(list.Dispose(inputDeps), slices.Dispose(inputDeps));

		/// <summary> Provides access to slice by index. </summary>
		public NativeArray<T> this[int sliceIndex]
		{
			get
			{
				int sliceBegin = sliceIndex == 0 ? 0 : slices[sliceIndex - 1];
				int sliceEnd = sliceIndex < slices.Length ? slices[sliceIndex] : list.Length;
				return list.AsArray().GetSubArray(sliceBegin, sliceEnd - sliceBegin);
			}
		}

		/// <summary> Provides access to item in slice by indices. </summary>
		public T this[int sliceIndex, int itemIndex]
		{
			get
			{
				int sliceBegin = sliceIndex == 0 ? 0 : slices[sliceIndex - 1];
				return list[sliceBegin + itemIndex];
			}
			set
			{
				int sliceBegin = sliceIndex == 0 ? 0 : slices[sliceIndex - 1];
				list[sliceBegin + itemIndex] = value;
			}
		}

		public void EnsureItemsCapacity(int itemsCapacity)
			=> list.EnsureCapacity(itemsCapacity);

		/// <summary> Finishes current slice and adds new one. </summary>
		public void Add(IEnumerable<T> slice)
		{
			FinishSlice();
			foreach (var item in slice)
				list.Add(item);
		}

		/// <inheritdoc cref="Add(IEnumerable{T})" />
		public void Add(IReadOnlyCollection<T> slice)
		{
			FinishSlice();
			list.EnsureCapacity(list.Capacity + slice.Count);
			foreach (var item in slice)
				list.Add(item);
		}

		/// <inheritdoc cref="Add(IEnumerable{T})" />
		public void Add(NativeArray<T> slice)
		{
			FinishSlice();
			list.AddRange(slice);
		}

		/// <inheritdoc cref="Add(IEnumerable{T})" />
		public void Add(NativeList<T> slice)
		{
			FinishSlice();
			list.AddRange(slice.AsArray());
		}

		/// <summary> Adds item to the last slice. </summary>
		/// <param name="item"></param>
		public void AddLastSliceItem(T item)
			=> list.Add(item);

		/// <summary> Adds new slice bound if last slice is not empty. </summary>
		public void FinishSlice()
		{
			if (slices.IsEmpty ? !list.IsEmpty : slices[^1] != list.Length)
				slices.Add(list.Length);
		}

		public void Clear()
		{
			throw new NotImplementedException();
		}

		public NativeArray<T> AsArray() => list.AsArray();

		public IEnumerator<NativeArray<T>> GetEnumerator()
			=> new SlicesEnumerator(this);

		IEnumerator IEnumerable.GetEnumerator()
			=> new SlicesEnumerator(this);

		private class SlicesEnumerator : IEnumerator<NativeArray<T>>
		{
			private NativeSlicedList<T> slicedList;
			private int sliceIndex;

			public NativeArray<T> Current => slicedList[sliceIndex];

			object IEnumerator.Current => Current;

			public SlicesEnumerator(NativeSlicedList<T> slicedList)
			{
				this.slicedList = slicedList;
				this.sliceIndex = -1;
			}

			public bool MoveNext()
				=> ++sliceIndex < slicedList.SlicesCount;

			public void Reset()
				=> sliceIndex = -1;

			public void Dispose()
			{
			}
		}

		private static int GetItemsCount(IReadOnlyCollection<NativeArray<T>> slices)
		{
			int count = 0;
			foreach (var slice in slices)
				count += slice.Length;
			return count;
		}

		private static int GetItemsCount(IReadOnlyCollection<NativeList<T>> slices)
		{
			int count = 0;
			foreach (var slice in slices)
				count += slice.Length;
			return count;
		}
	}
}