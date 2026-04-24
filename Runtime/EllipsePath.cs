using System;
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
	public static class EllipsePath
	{
    public static NativeArray<float2> CreateArray(float2 center, float2 radius, float segmentsDensity, Allocator allocator = Allocator.Temp)
    {
      var result = new NativeArray<float2>(GetSegmentsCount(radius, segmentsDensity), allocator, NativeArrayOptions.UninitializedMemory);
      Fill(center, radius, result);
      return result;
    }

    public static NativeArray<int2> CreateArray(int2 center, int2 radius, float segmentsDensity, Allocator allocator = Allocator.Temp)
    {
      var result = new NativeArray<int2>(GetSegmentsCount(radius, segmentsDensity), allocator, NativeArrayOptions.UninitializedMemory);
      Fill(center, radius, result);
      return result;
    }

    public static int GetSegmentsCount(float radius, float segmentsDensity)
      => (int)(segmentsDensity * math.sqrt(radius) + 3);

    public static int GetSegmentsCount(float2 radius, float segmentsDensity)
      => (int)(segmentsDensity * math.sqrt((radius.x + radius.y) * .5f) + 3);

    public static void Fill(float2 center, float2 radius, Span<float2> result)
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

    public static void Fill(int2 center, int2 radius, Span<int2> result)
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

    public static void Fill(float2 center, float2 radius, NativeArray<float2> result)
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

    public static void Fill(
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