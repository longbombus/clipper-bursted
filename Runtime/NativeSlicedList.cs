using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Clipper
{
	/// <summary>
	/// This collection represent a list of sequences (slices) of items.
	/// Empty sequences are not allowed.
	/// You can add items to the last slice until you call <see cref="FinishSlice"/> method, which will start new slice sequence.
	/// </summary>
	[BurstCompile]
	public struct NativeSlicedList<T> : INativeDisposable, IEnumerable<NativeArray<T>>
		where T : unmanaged
	{
		private NativeList<T> items;
		private NativeList<int> slices;

		/// <summary> Number of items in all slices. </summary>
		public int ItemsCount => items.Length;

		/// <summary> Number of items in all slices. </summary>
		public int ItemsCapacity => items.Capacity;

		/// <summary> Number of slices. </summary>
		public int SlicesCount
		{
			get
			{
				if (slices.Length == 0)
					return items.Length > 0 ? 1 : 0;
				else
					return items.Length > slices[^1] ? slices.Length + 1 : slices.Length;
			}
		}

		[Obsolete("Use SlicesCount property instead")]
		public int Count => SlicesCount;

		public bool IsCreated => items.IsCreated && slices.IsCreated;

		public NativeSlicedList(Allocator allocator)
		{
			items = new NativeList<T>(allocator);
			slices = new NativeList<int>(allocator);
		}

		public NativeSlicedList(int itemsCapacity, Allocator allocator)
		{
			items = new NativeList<T>(itemsCapacity, allocator);
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
			items.Dispose();
			slices.Dispose();
		}

		public JobHandle Dispose(JobHandle inputDeps)
			=> JobHandle.CombineDependencies(items.Dispose(inputDeps), slices.Dispose(inputDeps));

		/// <summary> Provides access to slice by index. </summary>
		public NativeArray<T> this[int sliceIndex]
		{
			get
			{
				int sliceBegin = sliceIndex == 0 ? 0 : slices[sliceIndex - 1];
				int sliceEnd = sliceIndex < slices.Length ? slices[sliceIndex] : items.Length;
				return items.AsArray().GetSubArray(sliceBegin, sliceEnd - sliceBegin);
			}
		}

		/// <summary> Provides access to item in slice by indices. </summary>
		public T this[int sliceIndex, int itemIndex]
		{
			get
			{
				int sliceBegin = sliceIndex == 0 ? 0 : slices[sliceIndex - 1];
				return items[sliceBegin + itemIndex];
			}
			set
			{
				int sliceBegin = sliceIndex == 0 ? 0 : slices[sliceIndex - 1];
				items[sliceBegin + itemIndex] = value;
			}
		}

		public void EnsureItemsCapacity(int itemsCapacity)
			=> items.EnsureCapacity(itemsCapacity);

		/// <summary> Finishes current slice and adds new one. </summary>
		public void Add(IEnumerable<T> slice)
		{
			FinishSlice();
			foreach (var item in slice)
				items.Add(item);
		}

		/// <inheritdoc cref="Add(IEnumerable{T})" />
		public void Add(IReadOnlyCollection<T> slice)
		{
			FinishSlice();
			items.EnsureCapacity(items.Length + slice.Count);
			foreach (var item in slice)
				items.Add(item);
		}

		/// <inheritdoc cref="Add(IEnumerable{T})" />
		public void Add(NativeArray<T> slice)
		{
			FinishSlice();
			items.AddRange(slice);
		}

		/// <inheritdoc cref="Add(IEnumerable{T})" />
		public void Add(NativeList<T> slice)
		{
			FinishSlice();
			items.AddRange(slice.AsArray());
		}

		/// <summary> Adds item to the last slice. </summary>
		/// <param name="item"></param>
		public void AddLastSliceItem(T item)
			=> items.Add(item);

		public NativeArray<T> AddLastSliceItems(int count)
		{
			int oldLength = items.Length;
			items.ResizeUninitialized(oldLength + count);
			return items.AsArray().GetSubArray(oldLength, count);
		}

		/// <summary> Adds new slice bound if last slice is not empty. </summary>
		public void FinishSlice()
		{
			if (slices.IsEmpty ? !items.IsEmpty : slices[^1] != items.Length)
				slices.Add(items.Length);
		}

		/// <summary> Clears all slices and items. </summary>
		public void Clear()
		{
			items.Clear();
			slices.Clear();
		}

		public NativeArray<T> AsArray() => items.AsArray();
		public NativeList<T> AsList() => items;

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