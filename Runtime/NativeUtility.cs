using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

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
		public static void AddRange(this NativeList<int2> list, NativeArray<float2> items, float scale)
		{
			var oldLength = list.Length;
			list.ResizeUninitialized(oldLength + items.Length);
			for (int i = 0; i < items.Length; ++i)
				list[oldLength + i] = (int2)(items[i] * scale);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void AddRange(this NativeList<float2> list, NativeArray<int2> items, float scale)
		{
			var oldLength = list.Length;
			list.ResizeUninitialized(oldLength + items.Length);
			for (int i = 0; i < items.Length; ++i)
				list[oldLength + i] = (float2)items[i] * scale;
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Add(this NativeSlicedList<int2> list, NativeArray<float2> slice, float scale)
		{
			list.FinishSlice();
			list.AddRange(slice, scale);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void AddRange(this NativeSlicedList<int2> list, NativeArray<float2> slice, float scale)
		{
			var buffer = list.AddLastSliceItems(slice.Length);
			for (int i = 0; i < slice.Length; ++i)
				buffer[i] = (int2)(slice[i] * scale);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Add(this NativeSlicedList<float2> list, NativeArray<int2> items, float scale)
		{
			list.FinishSlice();
			list.AddRange(items, scale);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void AddRange(this NativeSlicedList<float2> list, NativeArray<int2> items, float scale)
		{
			var buffer = list.AddLastSliceItems(items.Length);
			for (int i = 0; i < items.Length; ++i)
				buffer[i] = (float2)items[i] * scale;
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void AddRange<T>(this NativeList<T> list, NativeArray<T> items, NativeBitArray filter)
			where T : unmanaged
		{
			list.EnsureCapacity(list.Length + items.Length);
			for (int i = 0; i < items.Length; ++i)
				if (filter.IsSet(i))
					list.Add(items[i]);
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Negate(this NativeBitArray bits)
		{
			for (int i = 0; i < bits.Length; ++i)
				bits.Set(i, !bits.IsSet(i));
		}
	}
}