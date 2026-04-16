using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;

namespace Clipper
{
	[BurstCompile]
	public static class NativeUtility
	{
		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Reverse<T>(this NativeArray<T> arr)
			where T : struct
		{
			int lengthHalf = arr.Length / 2;
			int lastIndex = arr.Length - 1;
			for (int i = 0; i < lengthHalf; ++i)
			{
				int j = lastIndex - i;
				(arr[i], arr[j]) = (arr[j], arr[i]);
			}
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void EnsureCapacity<T>(this NativeList<T> list, int capacity)
			where T : unmanaged
		{
			if (list.Capacity < capacity)
				list.SetCapacity(capacity);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void AddRange(this NativeList<int> list, NativeArray<float> items, float scale)
		{
			var oldLength = list.Length;
			list.ResizeUninitialized(oldLength + items.Length);
			for (int i = 0; i < items.Length; ++i)
				list[oldLength + i] = (int)(items[i] * scale);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Add(this NativeSlicedList<int> list, NativeArray<float> slice, float scale)
		{
			list.FinishSlice();
			list.AddRange(slice, scale);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void AddRange(this NativeSlicedList<int> list, NativeArray<float> slice, float scale)
		{
			list.EnsureItemsCapacity(list.ItemsCount + slice.Length);
			for (int i = 0; i < slice.Length; ++i)
				list.AddLastSliceItem((int)(slice[i] * scale));
		}
	}
}