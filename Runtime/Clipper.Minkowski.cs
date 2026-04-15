/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  10 October 2024                                                 *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2024                                         *
* Purpose   :  Minkowski Sum and Difference                                    *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using Unity.Mathematics;
using Unity.Collections;

namespace Clipper
{
  public static class Minkowski
  {
    // Internal helper to compute signed area of a quad (a,b,c,d)
    private static long QuadSignedArea(in int2 a, in int2 b, in int2 c, in int2 d)
    {
      // Use 64-bit to avoid overflow for large coordinates
      long area = 0;
      // shoelace: sum over edges (x_i * y_{i+1} - x_{i+1} * y_i)
      area += (long)a.x * b.y - (long)b.x * a.y;
      area += (long)b.x * c.y - (long)c.x * b.y;
      area += (long)c.x * d.y - (long)d.x * c.y;
      area += (long)d.x * a.y - (long)a.x * d.y;
      return area;
    }

    // MinkowskiInternal: pattern & path are input NativeArray<int2> (caller-owned).
    // Returns a SlicedList<int2> allocated with 'allocator' that contains quads (4 points per slice)
    public static SlicedList<int2> MinkowskiInternal(NativeArray<int2> pattern, NativeArray<int2> path, bool isSum, bool isClosed, Allocator allocator)
    {
      int delta = isClosed ? 0 : 1;
      int patLen = pattern.Length;
      int pathLen = path.Length;

      SlicedList<int2> tmp = new SlicedList<int2>(allocator);

      // For each point in 'path', add a slice containing pattern translated by that point
      for (int pi = 0; pi < pathLen; ++pi)
      {
        int2 pathPt = path[pi];
        for (int k = 0; k < patLen; ++k)
        {
          int2 basePt = pattern[k];
          tmp.AddItem(isSum ? new int2(pathPt.x + basePt.x, pathPt.y + basePt.y)
                           : new int2(pathPt.x - basePt.x, pathPt.y - basePt.y));
        }
        tmp.AddSlice();
      }

      // Build result: for each adjacent pair of slices produce quads for each pattern edge
      SlicedList<int2> result = new SlicedList<int2>(allocator);
      if (pathLen == 0 || patLen == 0) return result;

      int g = isClosed ? pathLen - 1 : 0;
      int h = patLen - 1;
      for (int i = delta; i < pathLen; ++i)
      {
        NativeSlice<int2> sliceG = tmp.GetSlice(g);
        NativeSlice<int2> sliceI = tmp.GetSlice(i);
        for (int j = 0; j < patLen; ++j)
        {
          int2 p0 = sliceG[h];
          int2 p1 = sliceI[h];
          int2 p2 = sliceI[j];
          int2 p3 = sliceG[j];

          // Ensure positive winding (clockwise positive in original code)
          long area = QuadSignedArea(p0, p1, p2, p3);
          if (area == 0)
          {
            // degenerate quad - skip
            h = j;
            continue;
          }

          if (area < 0)
          {
            // reverse ordering to maintain consistent orientation
            result.AddItem(p0);
            result.AddItem(p3);
            result.AddItem(p2);
            result.AddItem(p1);
          }
          else
          {
            result.AddItem(p0);
            result.AddItem(p1);
            result.AddItem(p2);
            result.AddItem(p3);
          }
          result.AddSlice();

          h = j;
        }
        g = i;
      }

      tmp.Dispose();
      return result;
    }

    // Public API: inputs are NativeArray<int2>, output is SlicedList<int2> allocated with 'allocator'
    public static SlicedList<int2> Sum(NativeArray<int2> pattern, NativeArray<int2> path, bool isClosed, Allocator allocator)
    {
      SlicedList<int2> raw = MinkowskiInternal(pattern, path, true, isClosed, allocator);
      // The Minkowski result historically was unioned using Clipper.Union(...).
      // We will convert the SlicedList into PathsI, call Union, then convert back to SlicedList.
      // To avoid extra allocations here and keep changes minimal, create a PathsI, call Union, then convert.

      // Build PathsI from raw
      PathsI tmpPaths = new PathsI(raw.SliceCount);
      for (int si = 0; si < raw.SliceCount; ++si)
      {
        var slice = raw.GetSlice(si);
        PathI p = new PathI(slice.Length);
        for (int k = 0; k < slice.Length; ++k) p.Add(slice[k]);
        tmpPaths.Add(p);
      }
      raw.Dispose();

      PathsI unioned = Clipper.Union(tmpPaths, FillRule.NonZero);

      // Convert unioned PathsI back into SlicedList<int2> with the requested allocator
      SlicedList<int2> result = new SlicedList<int2>(allocator);
      for (int i = 0; i < unioned.Count; ++i)
      {
        foreach (var pt in unioned[i]) result.AddItem(pt);
        result.AddSlice();
      }
      return result;
    }

    public static SlicedList<int2> Sum(NativeArray<float2> pattern, NativeArray<float2> path, bool isClosed, int decimalPlaces, Allocator allocator)
    {
      float scale = decimalPlaces <= 0 ? 1f : math.exp10(math.min(decimalPlaces, 8));
      int plen = pattern.Length;
      int tlen = path.Length;
      // create temporary scaled int arrays (Temp)
      NativeArray<int2> patI = new NativeArray<int2>(plen, Allocator.Temp);
      NativeArray<int2> pathI = new NativeArray<int2>(tlen, Allocator.Temp);
      for (int i = 0; i < plen; ++i) patI[i] = (int2)math.round(pattern[i] * scale);
      for (int i = 0; i < tlen; ++i) pathI[i] = (int2)math.round(path[i] * scale);

      var res = Sum(patI, pathI, isClosed, allocator);

      patI.Dispose();
      pathI.Dispose();
      return res;
    }

    public static SlicedList<int2> Diff(NativeArray<int2> pattern, NativeArray<int2> path, bool isClosed, Allocator allocator)
    {
      // similar to Sum but isSum=false
      SlicedList<int2> raw = MinkowskiInternal(pattern, path, false, isClosed, allocator);

      PathsI tmpPaths = new PathsI(raw.SliceCount);
      for (int si = 0; si < raw.SliceCount; ++si)
      {
        var slice = raw.GetSlice(si);
        PathI p = new PathI(slice.Length);
        for (int k = 0; k < slice.Length; ++k) p.Add(slice[k]);
        tmpPaths.Add(p);
      }
      raw.Dispose();

      PathsI unioned = Clipper.Union(tmpPaths, FillRule.NonZero);

      SlicedList<int2> result = new SlicedList<int2>(allocator);
      for (int i = 0; i < unioned.Count; ++i)
      {
        foreach (var pt in unioned[i]) result.AddItem(pt);
        result.AddSlice();
      }
      return result;
    }

    public static SlicedList<int2> Diff(NativeArray<float2> pattern, NativeArray<float2> path, bool isClosed, int decimalPlaces, Allocator allocator)
    {
      float scale = decimalPlaces <= 0 ? 1f : math.exp10(math.min(decimalPlaces, 8));
      int plen = pattern.Length;
      int tlen = path.Length;
      NativeArray<int2> patI = new NativeArray<int2>(plen, Allocator.Temp);
      NativeArray<int2> pathI = new NativeArray<int2>(tlen, Allocator.Temp);
      for (int i = 0; i < plen; ++i) patI[i] = (int2)math.round(pattern[i] * scale);
      for (int i = 0; i < tlen; ++i) pathI[i] = (int2)math.round(path[i] * scale);

      var res = Diff(patI, pathI, isClosed, allocator);

      patI.Dispose();
      pathI.Dispose();
      return res;
    }

  }

} // namespace