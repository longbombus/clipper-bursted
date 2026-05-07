using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
  [BurstCompile]
	public static class PathUtility
	{
    [BurstCompile]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Centrize(ref NativeArray<float2> path, out float2 oldCenter)
    {
      if (path.Length == 0)
      {
        oldCenter = default;
        return;
      }

      oldCenter = path[0];
      for (int i = 1; i < path.Length; ++i)
        oldCenter += path[i];

      oldCenter /= path.Length;

      for (int i = 1; i < path.Length; ++i)
        path[i] -= oldCenter;
    }

    public static NativeArray<float2> CreateEllipseArray(float2 center, float2 radius, float segmentsDensity, Allocator allocator = Allocator.Temp)
    {
      var result = new NativeArray<float2>(GetEllipseSegmentsCount(radius, segmentsDensity), allocator, NativeArrayOptions.UninitializedMemory);
      FillEllipse(center, radius, result);
      return result;
    }

    public static NativeArray<int2> CreateEllipseArray(int2 center, int2 radius, float segmentsDensity, Allocator allocator = Allocator.Temp)
    {
      var result = new NativeArray<int2>(GetEllipseSegmentsCount(radius, segmentsDensity), allocator, NativeArrayOptions.UninitializedMemory);
      FillEllipse(center, radius, result);
      return result;
    }

    public static int GetEllipseSegmentsCount(float radius, float segmentsDensity)
      => (int)(segmentsDensity * math.sqrt(radius) + 3);

    public static int GetEllipseSegmentsCount(float2 radius, float segmentsDensity)
      => (int)(segmentsDensity * math.sqrt((radius.x + radius.y) * .5f) + 3);

    public static void FillEllipse(float2 center, float2 radius, Span<float2> result)
    {
      result[0] = new float2(center.x + radius.x, center.y);
      math.sincos(math.TAU / result.Length, out var s, out var c);
      float2x2 rot = new float2x2(c, -s, s, c);
      float2 d = new float2(c, s);
      for (int i = 1; i < result.Length; ++i)
      {
        result[i] = math.mad(d, radius, center);
        d = math.mul(rot, d);
      }
    }

    public static void FillEllipse(int2 center, int2 radius, Span<int2> result)
    {
      result[0] = new int2(center.x + radius.x, center.y);
      math.sincos(math.TAU / result.Length, out var s, out var c);
      float2x2 rot = new float2x2(c, -s, s, c);
      float2 d = new float2(c, s);
      for (int i = 1; i < result.Length; ++i)
      {
        result[i] = (int2)math.round(math.mad(d, radius, center));
        d = math.mul(rot, d);
      }
    }

    public static void FillEllipse(float2 center, float2 radius, NativeArray<float2> result)
    {
      result[0] = new float2(center.x + radius.x, center.y);
      math.sincos(math.TAU / result.Length, out var s, out var c);
      float2x2 rot = new float2x2(c, -s, s, c);
      float2 d = new float2(c, s);
      for (int i = 1; i < result.Length; ++i)
      {
        result[i] = math.mad(d, radius, center);
        d = math.mul(rot, d);
      }
    }

    public static void FillEllipse(
      int2 center,
      int2 radius,
      NativeArray<int2> result
    )
    {
      result[0] = new int2(center.x + radius.x, center.y);
      math.sincos(math.TAU / result.Length, out var s, out var c);
      float2x2 rot = new float2x2(c, -s, s, c);
      float2 d = new float2(c, s);
      for (int i = 1; i < result.Length; ++i)
      {
        result[i] = (int2)math.round(math.mad(d, radius, center));
        d = math.mul(rot, d);
      }
    }
	}
}