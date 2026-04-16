/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  14 December 2025                                                *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  This module contains simple functions that will likely cover    *
*              most polygon boolean and offsetting needs, while also avoiding  *
*              the inherent complexities of the other modules.                 *
* Thanks    :  Special thanks to Thong Nguyen, Guus Kuiper, Phil Stopford,     *
*           :  and Daniel Gosnell for their invaluable assistance with C#.     *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
  public static class Clipper
  {
    public static int4 InvalidRectI => new int4(int.MaxValue, int.MaxValue, int.MinValue, int.MinValue);
    public static readonly float4 InvalidRectF = new float4(float.MaxValue, float.MaxValue, float.MinValue, float.MinValue);

    public static PathsI Intersect(PathsI subject, PathsI clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Intersection, subject, clip, fillRule);
    }

    public static PathsF Intersect(PathsF subject, PathsF clip,
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Intersection,
        subject, clip, fillRule, precision);
    }

    public static PathsI Union(PathsI subject, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, null, fillRule);
    }

    public static PathsI Union(PathsI subject, PathsI clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, clip, fillRule);
    }

    public static PathsF Union(PathsF subject, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, null, fillRule);
    }

    public static PathsF Union(PathsF subject, PathsF clip,
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Union,
        subject, clip, fillRule, precision);
    }

    public static PathsI Difference(PathsI subject, PathsI clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Difference, subject, clip, fillRule);
    }

    public static PathsF Difference(PathsF subject, PathsF clip,
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Difference,
        subject, clip, fillRule, precision);
    }

    public static PathsI Xor(PathsI subject, PathsI clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Xor, subject, clip, fillRule);
    }

    public static PathsF Xor(PathsF subject, PathsF clip,
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Xor,
        subject, clip, fillRule, precision);
    }

    public static PathsI BooleanOp(ClipType clipType,
      PathsI? subject, PathsI? clip, FillRule fillRule)
    {
      PathsI solution = new PathsI();
      if (subject == null) return solution;
      Clipper64 c = new Clipper64();
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, solution);
      return solution;
    }

    public static void BooleanOp(ClipType clipType,
      PathsI? subject, PathsI? clip,
      PolyTree64 polytree, FillRule fillRule)
    {
      if (subject == null) return;
      Clipper64 c = new Clipper64();
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, polytree);
    }

    public static PathsF BooleanOp(ClipType clipType, PathsF subject, PathsF? clip,
      FillRule fillRule, int precision = 2)
    {
      PathsF solution = new PathsF();
      ClipperD c = new ClipperD(precision);
      c.AddSubject(subject);
      if (clip != null)
        c.AddClip(clip);
      c.Execute(clipType, fillRule, solution);
      return solution;
    }

    public static void BooleanOp(ClipType clipType,
      PathsF? subject, PathsF? clip,
      PolyTreeD polytree, FillRule fillRule, int precision = 2)
    {
      if (subject == null) return;
      ClipperD c = new ClipperD(precision);
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, polytree);
    }

    public static PathsI InflatePaths(
      PathsI paths,
      float delta,
      JoinType joinType,
      EndType endType,
      float miterLimit = 2f,
      float arcTolerance = 0f
    )
    {
      ClipperOffset co = new ClipperOffset(miterLimit, arcTolerance);
      co.AddPaths(paths, joinType, endType);
      PathsI solution = new PathsI();
      co.Execute(delta, solution);
      return solution;
    }

    public static PathsF InflatePaths(
      PathsF paths,
      float delta,
      JoinType joinType,
      EndType endType,
      float miterLimit = 2f,
      int precision = 2,
      float arcTolerance = 0f
    )
    {
      float scale = InternalClipper.PrecisionToScale(precision);
      PathsI tmp = ScalePaths64(paths, scale);
      ClipperOffset co = new ClipperOffset(miterLimit, scale * arcTolerance);
      co.AddPaths(tmp, joinType, endType);
      co.Execute(delta * scale, tmp); // reuse 'tmp' to receive (scaled) solution
      return ScalePathsD(tmp, 1 / scale);
    }

    public static PathsI RectClip(int4 rect, PathsI paths)
    {
      if (rect.IsEmpty() || paths.Count == 0) return new PathsI();
      RectClip64 rc = new RectClip64(rect);
      return rc.Execute(paths);
    }

    public static PathsI RectClip(int4 rect, PathI path)
    {
      if (rect.IsEmpty() || path.Count == 0) return new PathsI();
      PathsI tmp = new PathsI { path };
      return RectClip(rect, tmp);
    }

    public static PathsF RectClip(float4 rect, PathsF paths, int precision = 2)
    {
      if (rect.IsEmpty() || paths.Count == 0) return new PathsF();
      float scale = InternalClipper.PrecisionToScale(precision);
      int4 r = ScaleRect(rect, scale);
      PathsI tmpPath = ScalePaths64(paths, scale);
      RectClip64 rc = new RectClip64(r);
      tmpPath = rc.Execute(tmpPath);
      return ScalePathsD(tmpPath, 1 / scale);
    }

    public static PathsF RectClip(float4 rect, PathF path, int precision = 2)
    {
      if (rect.IsEmpty() || path.Count == 0) return new PathsF();
      PathsF tmp = new PathsF { path };
      return RectClip(rect, tmp, precision);
    }
    public static PathsI RectClipLines(int4 rect, PathsI paths)
    {
      if (rect.IsEmpty() || paths.Count == 0) return new PathsI();
      RectClipLines64 rc = new RectClipLines64(rect);
      return rc.Execute(paths);
    }

    public static PathsI RectClipLines(int4 rect, PathI path)
    {
      if (rect.IsEmpty() || path.Count == 0) return new PathsI();
      PathsI tmp = new PathsI { path };
      return RectClipLines(rect, tmp);
    }

    public static PathsF RectClipLines(float4 rect,
      PathsF paths, int precision = 2)
    {
      if (rect.IsEmpty() || paths.Count == 0) return new PathsF();
      float scale = InternalClipper.PrecisionToScale(precision);
      int4 r = ScaleRect(rect, scale);
      PathsI tmpPath = ScalePaths64(paths, scale);
      RectClipLines64 rc = new RectClipLines64(r);
      tmpPath = rc.Execute(tmpPath);
      return ScalePathsD(tmpPath, 1 / scale);
    }
    public static PathsF RectClipLines(float4 rect, PathF path, int precision = 2)
    {
      if (rect.IsEmpty() || path.Count == 0) return new PathsF();
      PathsF tmp = new PathsF { path };
      return RectClipLines(rect, tmp, precision);
    }
    public static PathsI MinkowskiSum(PathI pattern, PathI path, bool isClosed)
    {
      return Minkowski.Sum(pattern, path, isClosed);
    }

    public static PathsF MinkowskiSum(PathF pattern, PathF path, bool isClosed)
    {
      return Minkowski.Sum(pattern, path, isClosed);
    }

    public static PathsI MinkowskiDiff(PathI pattern, PathI path, bool isClosed)
    {
      return Minkowski.Diff(pattern, path, isClosed);
    }

    public static PathsF MinkowskiDiff(PathF pattern, PathF path, bool isClosed)
    {
      return Minkowski.Diff(pattern, path, isClosed);
    }

    public static float Area(PathI path)
    {
      // https://en.wikipedia.org/wiki/Shoelace_formula
      float a = 0f;
      if (path.Count < 3)
        return 0f;

      int2 prevPt = path[^1];
      foreach (int2 pt in path)
      {
        a += (float)(prevPt.y + pt.y) * (prevPt.x - pt.x);
        prevPt = pt;
      }
      return a * 0.5f;
    }

    public static float Area(PathsI paths)
    {
      float a = 0f;
      foreach (var path in paths)
        a += Area(path);
      return a;
    }

    public static float Area(PathF path)
    {
      float a = 0f;
      if (path.Count < 3)
        return 0f;

      float2 prevPt = path[^1];
      foreach (float2 pt in path)
      {
        a += (prevPt.y + pt.y) * (prevPt.x - pt.x);
        prevPt = pt;
      }
      return a * 0.5f;
    }

    public static double Area(PathsF paths)
    {
      double a = 0.0;
      foreach (var path in paths)
        a += Area(path);
      return a;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPositive(PathI poly)
    {
      return Area(poly) >= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPositive(PathF poly)
    {
      return Area(poly) >= 0;
    }

    public static PathI OffsetPath(PathI path, int dx, int dy)
    {
      PathI result = new PathI(path.Count);
      foreach (int2 pt in path)
        result.Add(new int2(pt.x + dx, pt.y + dy));
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 Scaleint2(int2 pt, float scale)
      => (int2) math.round((float2)pt * scale);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 Scalefloat2(int2 pt, float scale)
      => (float2)pt * scale;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int4 ScaleRect(float4 rec, float scale)
      => (int4)(rec * scale);

    // Unlike ScalePath, both ScalePath64 & ScalePathD also involve type conversion
    public static PathI ScalePath64(PathF path, float scale)
    {
      int cnt = path.Count;
      PathI res = new PathI(cnt);
      foreach (float2 pt in path)
        res.Add((int2)(pt * scale));
      return res;
    }

    public static PathsI ScalePaths64(PathsF paths, float scale)
    {
      int cnt = paths.Count;
      PathsI res = new PathsI(cnt);
      foreach (var path in paths)
        res.Add(ScalePath64(path, scale));
      return res;
    }

    public static PathF ScalePathD(PathI path, float scale)
    {
      int cnt = path.Count;
      PathF res = new PathF(cnt);
      foreach (int2 pt in path)
        res.Add((float2)pt * scale);
      return res;
    }

    public static PathsF ScalePathsD(PathsI paths, float scale)
    {
      int cnt = paths.Count;
      PathsF res = new PathsF(cnt);
      foreach (var path in paths)
        res.Add(ScalePathD(path, scale));
      return res;
    }

    // The static functions Path64 and PathD convert path types without scaling
    public static PathI Path64(PathF path)
    {
      PathI result = new PathI(path.Count);
      foreach (float2 pt in path)
        result.Add(new int2(pt));
      return result;
    }

    public static PathsI Paths64(PathsF paths)
    {
      PathsI result = new PathsI(paths.Count);
      foreach (var path in paths)
        result.Add(Path64(path));
      return result;
    }

    public static PathsF PathsD(PathsI paths)
    {
      PathsF result = new PathsF(paths.Count);
      foreach (var path in paths)
        result.Add(PathD(path));
      return result;
    }

    public static PathF PathD(PathI path)
    {
      PathF result = new PathF(path.Count);
      foreach (int2 pt in path)
        result.Add(new float2(pt));
      return result;
    }

    public static PathI TranslatePath(PathI path, int dx, int dy)
    {
      PathI result = new PathI(path.Count);
      foreach (int2 pt in path)
        result.Add(new int2(pt.x + dx, pt.y + dy));
      return result;
    }

    public static PathsI TranslatePaths(PathsI paths, int dx, int dy)
    {
      PathsI result = new PathsI(paths.Count);
      foreach (var path in paths)
        result.Add(OffsetPath(path, dx, dy));
      return result;
    }

    public static PathF TranslatePath(PathF path, float dx, float dy)
    {
      PathF result = new PathF(path.Count);
      foreach (float2 pt in path)
        result.Add(new float2(pt.x + dx, pt.y + dy));
      return result;
    }

    public static PathsF TranslatePaths(PathsF paths, float dx, float dy)
    {
      PathsF result = new PathsF(paths.Count);
      foreach (var path in paths)
        result.Add(TranslatePath(path, dx, dy));
      return result;
    }

    public static PathI ReversePath(PathI path)
    {
      PathI result = new PathI(path);
      result.Reverse();
      return result;
    }

    public static int4 GetBounds(PathI path)
    {
      int4 result = InvalidRectI;
      foreach (int2 pt in path)
      {
        if (pt.x < result.x) result.x = pt.x;
        if (pt.x > result.z) result.z = pt.x;
        if (pt.y < result.y) result.y = pt.y;
        if (pt.y > result.w) result.w = pt.y;
      }
      return result.IsValid() ? result : new int4();
    }

    public static int4 GetBounds(PathsI paths)
    {
      int4 result = InvalidRectI;
      foreach (var path in paths)
        foreach (int2 pt in path)
        {
          if (pt.x < result.x) result.x = pt.x;
          if (pt.x > result.z) result.z = pt.x;
          if (pt.y < result.y) result.y = pt.y;
          if (pt.y > result.w) result.w = pt.y;
        }
      return result.IsValid() ? result : new int4();
    }

    public static float4 GetBounds(PathF path)
    {
      float4 result = InvalidRectF;
      foreach (float2 pt in path)
      {
        if (pt.x < result.x) result.x = pt.x;
        if (pt.x > result.z) result.z = pt.x;
        if (pt.y < result.y) result.y = pt.y;
        if (pt.y > result.w) result.w = pt.y;
      }
      return result.IsValid() ? result : new float4();
    }

    public static float4 GetBounds(PathsF paths)
    {
      float4 result = InvalidRectF;
      foreach (var path in paths)
        foreach (float2 pt in path)
        {
          if (pt.x < result.x) result.x = pt.x;
          if (pt.x > result.z) result.z = pt.x;
          if (pt.y < result.y) result.y = pt.y;
          if (pt.y > result.w) result.w = pt.y;
        }
      return result.IsValid() ? result : new float4();
    }

    public static PathI MakePath(int[] arr)
    {
      int len = arr.Length / 2;
      PathI p = new PathI(len);
      for (int i = 0; i < len; i++)
        p.Add(new int2(arr[i * 2], arr[i * 2 + 1]));
      return p;
    }

    public static PathF MakePath(float[] arr)
    {
      int len = arr.Length / 2;
      PathF p = new PathF(len);
      for (int i = 0; i < len; i++)
        p.Add(new float2(arr[i * 2], arr[i * 2 + 1]));
      return p;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Sqr(float val)
    {
      return val * val;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointsNearEqual(float2 pt1, float2 pt2, double distanceSqrd)
    {
      return Sqr(pt1.x - pt2.x) + Sqr(pt1.y - pt2.y) < distanceSqrd;
    }

    public static PathF StripNearDuplicates(PathF path,
        double minEdgeLenSqrd, bool isClosedPath)
    {
      int cnt = path.Count;
      PathF result = new PathF(cnt);
      if (cnt == 0) return result;
      float2 lastPt = path[0];
      result.Add(lastPt);
      for (int i = 1; i < cnt; i++)
        if (!PointsNearEqual(lastPt, path[i], minEdgeLenSqrd))
        {
          lastPt = path[i];
          result.Add(lastPt);
        }

      if (isClosedPath && PointsNearEqual(lastPt, result[0], minEdgeLenSqrd))
      {
        result.RemoveAt(result.Count - 1);
      }

      return result;
    }

    public static PathI StripDuplicates(PathI path, bool isClosedPath)
    {
      int cnt = path.Count;
      PathI result = new PathI(cnt);
      if (cnt == 0) return result;
      int2 lastPt = path[0];
      result.Add(lastPt);
      for (int i = 1; i < cnt; i++)
        if (!lastPt.Equals(path[i]))
        {
          lastPt = path[i];
          result.Add(lastPt);
        }
      if (isClosedPath && lastPt.Equals(result[0]))
        result.RemoveAt(result.Count - 1);
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddPolyNodeToPaths(PolyPath64 polyPath, PathsI paths)
    {
      if (polyPath.Polygon!.Count > 0)
        paths.Add(polyPath.Polygon);
      for (int i = 0; i < polyPath.Count; i++)
        AddPolyNodeToPaths((PolyPath64) polyPath._childs[i], paths);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PathsI PolyTreeToPaths64(PolyTree64 polyTree)
    {
      PathsI result = new PathsI();
      for (int i = 0; i < polyTree.Count; i++)
        AddPolyNodeToPaths((PolyPath64) polyTree._childs[i], result);
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddPolyNodeToPathsD(PolyPathD polyPath, PathsF paths)
    {
      if (polyPath.Polygon!.Count > 0)
        paths.Add(polyPath.Polygon);
      for (int i = 0; i < polyPath.Count; i++)
        AddPolyNodeToPathsD((PolyPathD) polyPath._childs[i], paths);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PathsF PolyTreeToPathsD(PolyTreeD polyTree)
    {
      PathsF result = new PathsF();
      foreach (PolyPathD polyPathBase in polyTree)
      {
        PolyPathD p = (PolyPathD)polyPathBase;
        AddPolyNodeToPathsD(p, result);
      }

      return result;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpendicDistFromLineSqrd(float2 pt, float2 line1, float2 line2)
    {
      float a = pt.x - line1.x;
      float b = pt.y - line1.y;
      float c = line2.x - line1.x;
      float d = line2.y - line1.y;
      if (c == 0 && d == 0) return 0;
      return Sqr(a * d - c * b) / (c * c + d * d);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpendicDistFromLineSqrd(int2 pt, int2 line1, int2 line2)
    {
      float a = pt.x - line1.x;
      float b = pt.y - line1.y;
      float c = line2.x - line1.x;
      float d = line2.y - line1.y;
      if (c == 0 && d == 0) return 0;
      return Sqr(a * d - c * b) / (c * c + d * d);
    }

    internal static void RDP(PathI path, int begin, int end, double epsSqrd, List<bool> flags)
    {
      while (true)
      {
        int idx = 0;
        double max_d = 0;
        while (end > begin && path[begin].Equals(path[end])) flags[end--] = false;
        for (int i = begin + 1; i < end; ++i)
        {
          // PerpendicDistFromLineSqrd - avoids expensive Sqrt()
          double d = PerpendicDistFromLineSqrd(path[i], path[begin], path[end]);
          if (d <= max_d) continue;
          max_d = d;
          idx = i;
        }

        if (max_d <= epsSqrd) return;
        flags[idx] = true;
        if (idx > begin + 1) RDP(path, begin, idx, epsSqrd, flags);
        if (idx < end - 1)
        {
          begin = idx;
          continue;
        }

        break;
      }
    }

    public static PathI RamerDouglasPeucker(PathI path, float epsilon)
    {
      int len = path.Count;
      if (len < 5) return path;
      List<bool> flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
      RDP(path, 0, len - 1, Sqr(epsilon), flags);
      PathI result = new PathI(len);
      for (int i = 0; i < len; ++i)
        if (flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsI RamerDouglasPeucker(PathsI paths, float epsilon)
    {
      PathsI result = new PathsI(paths.Count);
      foreach (var path in paths)
        result.Add(RamerDouglasPeucker(path, epsilon));
      return result;
    }

    internal static void RDP(PathF path, int begin, int end, float epsSqrd, List<bool> flags)
    {
      while (true)
      {
        int idx = 0;
        float max_d = 0;
        while (end > begin && path[begin].Equals(path[end])) flags[end--] = false;
        for (int i = begin + 1; i < end; ++i)
        {
          // PerpendicDistFromLineSqrd - avoids expensive Sqrt()
          float d = PerpendicDistFromLineSqrd(path[i], path[begin], path[end]);
          if (d <= max_d) continue;
          max_d = d;
          idx = i;
        }

        if (max_d <= epsSqrd) return;
        flags[idx] = true;
        if (idx > begin + 1) RDP(path, begin, idx, epsSqrd, flags);
        if (idx < end - 1)
        {
          begin = idx;
          continue;
        }

        break;
      }
    }

    public static PathF RamerDouglasPeucker(PathF path, float epsilon)
    {
      int len = path.Count;
      if (len < 5) return path;
      List<bool> flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
      RDP(path, 0, len - 1, Sqr(epsilon), flags);
      PathF result = new PathF(len);
      for (int i = 0; i < len; ++i)
        if (flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsF RamerDouglasPeucker(PathsF paths, float epsilon)
    {
      PathsF result = new PathsF(paths.Count);
      foreach (var path in paths)
        result.Add(RamerDouglasPeucker(path, epsilon));
      return result;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetNext(int current, int high, ref bool[] flags)
    {
      ++current;
      while (current <= high && flags[current]) ++current;
      if (current <= high) return current;
      current = 0;
      while (flags[current]) ++current;
      return current;
    }

    private static int GetPrior(int current, int high, ref bool[] flags)
    {
      if (current == 0) current = high;
      else --current;
      while (current > 0 && flags[current]) --current;
      if (!flags[current]) return current;
      current = high;
      while (flags[current]) --current;
      return current;
    }

      public static PathI SimplifyPath(
        PathI path,
        float epsilon,
        bool isClosedPath = true
      )
    {
      int len = path.Count, high = len - 1;
      double epsSqr = Sqr(epsilon);
      if (len < 4) return path;

      bool[] flags = new bool[len];
      double[] dsq = new double[len];
      int curr = 0;

      if (isClosedPath)
      {
        dsq[0] = PerpendicDistFromLineSqrd(path[0], path[high], path[1]);
        dsq[high] = PerpendicDistFromLineSqrd(path[high], path[0], path[high - 1]);
      }
      else
      {
        dsq[0] = double.MaxValue;
        dsq[high] = double.MaxValue;
      }

      for (int i = 1; i < high; ++i)
        dsq[i] = PerpendicDistFromLineSqrd(path[i], path[i - 1], path[i + 1]);

      for (; ; )
      {
        if (dsq[curr] > epsSqr)
        {
          int start = curr;
          do
          {
            curr = GetNext(curr, high, ref flags);
          } while (curr != start && dsq[curr] > epsSqr);
          if (curr == start) break;
        }

        int prev = GetPrior(curr, high, ref flags);
        int next = GetNext(curr, high, ref flags);
        if (next == prev) break;

        int prior2;
        if (dsq[next] < dsq[curr])
        {
          prior2 = prev;
          prev = curr;
          curr = next;
          next = GetNext(next, high, ref flags);
        }
        else
          prior2 = GetPrior(prev, high, ref flags);

        flags[curr] = true;
        curr = next;
        next = GetNext(next, high, ref flags);
        if (isClosedPath || ((curr != high) && (curr != 0)))
          dsq[curr] = PerpendicDistFromLineSqrd(path[curr], path[prev], path[next]);
        if (isClosedPath || ((prev != 0) && (prev != high)))
          dsq[prev] = PerpendicDistFromLineSqrd(path[prev], path[prior2], path[curr]);
      }
      PathI result = new PathI(len);
      for (int i = 0; i < len; i++)
        if (!flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsI SimplifyPaths(
      PathsI paths,
      float epsilon,
      bool isClosedPaths = true
    )
    {
      PathsI result = new PathsI(paths.Count);
      foreach (var path in paths)
        result.Add(SimplifyPath(path, epsilon, isClosedPaths));
      return result;
    }

    public static PathF SimplifyPath(
      PathF path,
      float epsilon,
      bool isClosedPath = true
    )
    {
      int len = path.Count, high = len - 1;
      float epsSqr = Sqr(epsilon);
      if (len < 4) return path;

      bool[] flags = new bool[len];
      double[] dsq = new double[len];
      int curr = 0;
      if (isClosedPath)
      {
        dsq[0] = PerpendicDistFromLineSqrd(path[0], path[high], path[1]);
        dsq[high] = PerpendicDistFromLineSqrd(path[high], path[0], path[high - 1]);
      }
      else
      {
        dsq[0] = double.MaxValue;
        dsq[high] = double.MaxValue;
      }
      for (int i = 1; i < high; ++i)
        dsq[i] = PerpendicDistFromLineSqrd(path[i], path[i - 1], path[i + 1]);

      for (; ; )
      {
        if (dsq[curr] > epsSqr)
        {
          int start = curr;
          do
          {
            curr = GetNext(curr, high, ref flags);
          } while (curr != start && dsq[curr] > epsSqr);
          if (curr == start) break;
        }

        int prev = GetPrior(curr, high, ref flags);
        int next = GetNext(curr, high, ref flags);
        if (next == prev) break;

        int prior2;
        if (dsq[next] < dsq[curr])
        {
          prior2 = prev;
          prev = curr;
          curr = next;
          next = GetNext(next, high, ref flags);
        }
        else
          prior2 = GetPrior(prev, high, ref flags);

        flags[curr] = true;
        curr = next;
        next = GetNext(next, high, ref flags);
        if (isClosedPath || ((curr != high) && (curr != 0)))
          dsq[curr] = PerpendicDistFromLineSqrd(path[curr], path[prev], path[next]);
        if (isClosedPath || ((prev != 0) && (prev != high)))
          dsq[prev] = PerpendicDistFromLineSqrd(path[prev], path[prior2], path[curr]);
      }
      PathF result = new PathF(len);
      for (int i = 0; i < len; i++)
        if (!flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsF SimplifyPaths(
      PathsF paths,
      float epsilon,
      bool isClosedPath = true
    )
    {
      PathsF result = new PathsF(paths.Count);
      foreach (var path in paths)
        result.Add(SimplifyPath(path, epsilon, isClosedPath));
      return result;
    }

    public static PathI TrimCollinear(PathI path, bool isOpen = false)
    {
      int len = path.Count;
      int i = 0;
      if (!isOpen)
      {
        while (i < len - 1 &&
          InternalClipper.IsCollinear(path[len - 1], path[i], path[i + 1])) i++;
        while (i < len - 1 && InternalClipper.IsCollinear(path[len - 2], path[len - 1], path[i])) len--;
      }

      if (len - i < 3)
      {
        if (!isOpen || len < 2 || path[0].Equals(path[1]))
          return new PathI();
        return path;
      }

      PathI result = new PathI(len - i);
      int2 last = path[i];
      result.Add(last);
      for (i++; i < len - 1; i++)
      {
        if (InternalClipper.IsCollinear(last, path[i], path[i + 1])) continue;
        last = path[i];
        result.Add(last);
      }

      if (isOpen)
        result.Add(path[len - 1]);
      else if (!InternalClipper.IsCollinear(last, path[len - 1], result[0]))
        result.Add(path[len - 1]);
      else
      {
        while (result.Count > 2 && InternalClipper.IsCollinear(
                 result[result.Count - 1], result[result.Count - 2], result[0]))
        {
          result.RemoveAt(result.Count - 1);
        }
        if (result.Count < 3)
          result.Clear();
      }
      return result;
    }

    public static PathF TrimCollinear(PathF path, int precision, bool isOpen = false)
    {
      float scale = InternalClipper.PrecisionToScale(precision);
      PathI p = ScalePath64(path, scale);
      p = TrimCollinear(p, isOpen);
      return ScalePathD(p, 1 / scale);
    }

    public static PointInPolygonResult PointInPolygon(int2 pt, PathI polygon)
    {
      return InternalClipper.PointInPolygon(pt, polygon);
    }

    public static PointInPolygonResult PointInPolygon(float2 pt,
      PathF polygon, int precision = 2)
    {
      float scale = InternalClipper.PrecisionToScale(precision);
      int2 p = (int2)(pt * scale);
      PathI path = ScalePath64(polygon, scale);
      return InternalClipper.PointInPolygon(p, path);
    }

    public static PathI Ellipse(
      int2 center,
      int2 radius,
      int segmentsCount = 0
    )
    {
      var result = new int2[segmentsCount];
      FillEllipse(center, radius, result);
      return new PathI(result);
    }

    public static PathF Ellipse(
      float2 center,
      float2 radius,
      int segmentsCount = 0
    )
    {
      var result = new float2[segmentsCount];
      FillEllipse(center, radius, result);
      return new PathF(result);
    }

    public static NativeArray<float2> CreateEllipseArray(
      float2 center,
      float2 radius,
      float segmentsDensity,
      Allocator allocator = Allocator.Temp
    )
    {
      var result = new NativeArray<float2>(GetEllipseSegmentsCount(radius, segmentsDensity), allocator, NativeArrayOptions.UninitializedMemory);
      FillEllipse(center, radius, result);
      return result;
    }

    public static NativeArray<int2> CreateEllipseArray(
      int2 center,
      int2 radius,
      float segmentsDensity,
      Allocator allocator = Allocator.Temp
    )
    {
      var result = new NativeArray<int2>(GetEllipseSegmentsCount(radius, segmentsDensity), allocator, NativeArrayOptions.UninitializedMemory);
      FillEllipse(center, radius, result);
      return result;
    }

    private static int GetEllipseSegmentsCount(float2 radius, float segmentsDensity)
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

    private static void ShowPolyPathStructure(PolyPath64 pp, int level)
    {
      string spaces = new string(' ', level * 2);
      string caption = (pp.IsHole ? "Hole " : "Outer ");
      if (pp.Count == 0)
      {
        Console.WriteLine(spaces + caption);
      }
      else
      {
        Console.WriteLine(spaces + caption + $"({pp.Count})");
        foreach (PolyPath64 child in pp) { ShowPolyPathStructure(child, level + 1); }
      }
    }

    public static void ShowPolyTreeStructure(PolyTree64 polytree)
    {
      Console.WriteLine("Polytree Root");
      foreach (PolyPath64 child in polytree) { ShowPolyPathStructure(child, 1); }
    }

    private static void ShowPolyPathStructure(PolyPathD pp, int level)
    {
      string spaces = new string(' ', level * 2);
      string caption = (pp.IsHole ? "Hole " : "Outer ");
      if (pp.Count == 0)
      {
        Console.WriteLine(spaces + caption);
      }
      else
      {
        Console.WriteLine(spaces + caption + $"({pp.Count})");
        foreach (PolyPathD child in pp) { ShowPolyPathStructure(child, level + 1); }
      }
    }

    public static void ShowPolyTreeStructure(PolyTreeD polytree)
    {
      Console.WriteLine("Polytree Root");
      foreach (PolyPathD child in polytree) { ShowPolyPathStructure(child, 1); }
    }

    public static TriangulateResult Triangulate(PathsI pp, out PathsI solution, bool useDelaunay = true)
    {
      Delaunay d = new Delaunay(useDelaunay);
      return d.Execute(pp, out solution);
    }

    public static TriangulateResult Triangulate(PathsF pp, int decPlaces, out PathsF solution, bool useDelaunay = true)
    {
      float scale = decPlaces <= 0 ? 1f : math.exp10(math.min(decPlaces, 8));

      PathsI pp64 = ScalePaths64(pp, scale);

      Delaunay d = new Delaunay(useDelaunay);
      TriangulateResult result = d.Execute(pp64, out PathsI sol64);
      if (result == TriangulateResult.success)
        solution = ScalePathsD(sol64, 1f / scale);
      else
        solution = new PathsF();
      return result;
    }

  } // Clipper
} // namespace