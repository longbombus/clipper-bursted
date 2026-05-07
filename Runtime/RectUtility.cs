using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
	[BurstCompile]
	public static class RectUtility
	{
		public static readonly int4 InvalidI = new int4(int.MaxValue, int.MaxValue, int.MinValue, int.MinValue);
		public static readonly float4 InvalidF = new float4(float.MaxValue, float.MaxValue, float.MinValue, float.MinValue);

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void GetBounds(this in NativeArray<int2> path, out int4 bounds)
		{
			bounds = InvalidI;
			foreach (int2 pt in path)
			{
				if (pt.x < bounds.x) bounds.x = pt.x;
				if (pt.x > bounds.z) bounds.z = pt.x;
				if (pt.y < bounds.y) bounds.y = pt.y;
				if (pt.y > bounds.w) bounds.w = pt.y;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void GetBounds(this NativeSlicedList<int2> paths, out int4 bounds)
			=> paths.AsArray().GetBounds(out bounds);

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void GetBounds(this in NativeSlice<int2> path, out int4 bounds)
		{
			bounds = InvalidI;
			foreach (int2 pt in path)
			{
				if (pt.x < bounds.x) bounds.x = pt.x;
				if (pt.x > bounds.z) bounds.z = pt.x;
				if (pt.y < bounds.y) bounds.y = pt.y;
				if (pt.y > bounds.w) bounds.w = pt.y;
			}
		}

		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void GetBounds(this in NativeArray<float2> path, out float4 bounds)
		{
			bounds = InvalidF;
			foreach (float2 pt in path)
			{
				if (pt.x < bounds.x) bounds.x = pt.x;
				if (pt.x > bounds.z) bounds.z = pt.x;
				if (pt.y < bounds.y) bounds.y = pt.y;
				if (pt.y > bounds.w) bounds.w = pt.y;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void GetBounds(this NativeSlicedList<float2> paths, float4 bounds)
			=> paths.AsArray().GetBounds(out bounds);
	}
}