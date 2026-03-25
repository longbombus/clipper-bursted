using System.Runtime.CompilerServices;
using Unity.Mathematics;

#if USINGZ
namespace Clipper2ZLib
#else
namespace Clipper2Lib
#endif
{
	#region Aliases
#if USINGZ
  using Point64 = int3;
  using PointD = float3;
#else
	using Point64 = int2;
	using PointD = float2;
#endif

	using Rect64 = int4;
	using RectD = float4;
	#endregion

	public static class VectorUtility
	{
		public const int MaxFloatInt = Huge * Huge;
		public const int Huge = 4096;
		public const float Tiny = 1f / Huge;

		#region Point

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Negate(this ref int2 v)
			=> v = -v;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Negate(this ref double2 v)
			=> v = -v;

		#endregion

		#region Rect

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static int GetWidth(this in int4 rect)
			=> rect.z - rect.x;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static int GetHeight(this in int4 rect)
			=> rect.w - rect.y;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void SetWidth(this ref int4 rect, int width)
			=> rect.z = rect.x + width;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void SetHeight(this ref int4 rect, int height)
			=> rect.w = rect.y + height;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool IsValid(this in int4 rect)
			=> rect.x < int.MaxValue;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool IsEmpty(this in int4 rect)
			=> rect.w <= rect.y || rect.z <= rect.x;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool IsEmpty(this in float4 rect)
			=> rect.w <= rect.y || rect.z <= rect.x;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static int2 MidPoint(this in int4 rect)
			=> (rect.xy + rect.zw) / 2;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float2 MidPoint(this in float4 rect)
			=> (rect.xy + rect.zw) * .5f;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool Contains(this in int4 rect, int2 point)
			=> math.all(new bool4(rect.xy < point, point < rect.zw));

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool Contains(this in float4 rect, float2 point)
			=> math.all(new bool4(rect.xy < point, point < rect.zw));

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool Contains(this in int4 outerRect, in int4 innerRect)
			=> math.all(new bool4(outerRect.xy <= innerRect.xy, innerRect.zw <= outerRect.zw));

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool Contains(this in float4 outerRect, in float4 innerRect)
			=> math.all(new bool4(outerRect.xy <= innerRect.xy, innerRect.zw <= outerRect.zw));

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool Intersects(this in int4 rect, in int4 other)
			=> (math.max(rect.x, other.x) <= math.min(rect.z, other.z))
			&& (math.max(rect.y, other.y) <= math.min(rect.w, other.w));

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool Intersects(this in float4 rect, in float4 other)
			=> (math.max(rect.x, other.x) <= math.min(rect.z, other.z))
			&& (math.max(rect.y, other.y) <= math.min(rect.w, other.w));

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void ToPath(this in int4 rect, Path64 path)
		{
			path.Add(rect.xy);
			path.Add(rect.zy);
			path.Add(rect.zw);
			path.Add(rect.xw);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void ToPath(this in float4 rect, PathD path)
		{
			path.Add(rect.xy);
			path.Add(rect.zy);
			path.Add(rect.zw);
			path.Add(rect.xw);
		}

		#endregion
	}
}