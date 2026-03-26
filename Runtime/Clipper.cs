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
using Unity.Mathematics;

namespace Clipper2Lib
{
  public static class Clipper
  {
    private static int4 invalidRectI = new int4(false);
    public static int4 InvalidRectI => invalidRectI;

    private static float4 invalidRectF = new float4(false);
    public static float4 InvalidRectF => invalidRectF;

    public static Paths64 Intersect(Paths64 subject, Paths64 clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Intersection, subject, clip, fillRule);
    }

    public static PathsD Intersect(PathsD subject, PathsD clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Intersection,
        subject, clip, fillRule, precision);
    }

    public static Paths64 Union(Paths64 subject, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, null, fillRule);
    }

    public static Paths64 Union(Paths64 subject, Paths64 clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, clip, fillRule);
    }

    public static PathsD Union(PathsD subject, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, null, fillRule);
    }

    public static PathsD Union(PathsD subject, PathsD clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Union,
        subject, clip, fillRule, precision);
    }

    public static Paths64 Difference(Paths64 subject, Paths64 clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Difference, subject, clip, fillRule);
    }

    public static PathsD Difference(PathsD subject, PathsD clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Difference,
        subject, clip, fillRule, precision);
    }

    public static Paths64 Xor(Paths64 subject, Paths64 clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Xor, subject, clip, fillRule);
    }

    public static PathsD Xor(PathsD subject, PathsD clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Xor, 
        subject, clip, fillRule, precision);
    }

    public static Paths64 BooleanOp(ClipType clipType,
      Paths64? subject, Paths64? clip, FillRule fillRule)
    {
      Paths64 solution = new Paths64();
      if (subject == null) return solution;
      Clipper64 c = new Clipper64();
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, solution);
      return solution;
    }

    public static void BooleanOp(ClipType clipType,
      Paths64? subject, Paths64? clip, 
      PolyTree64 polytree, FillRule fillRule)
    {
      if (subject == null) return;
      Clipper64 c = new Clipper64();
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, polytree);
    }

    public static PathsD BooleanOp(ClipType clipType, PathsD subject, PathsD? clip, 
      FillRule fillRule, int precision = 2)
    {
      PathsD solution = new PathsD();
      ClipperD c = new ClipperD(precision);
      c.AddSubject(subject);
      if (clip != null)
        c.AddClip(clip);
      c.Execute(clipType, fillRule, solution);
      return solution;
    }

    public static void BooleanOp(ClipType clipType,
      PathsD? subject, PathsD? clip,
      PolyTreeD polytree, FillRule fillRule, int precision = 2)
    {
      if (subject == null) return;
      ClipperD c = new ClipperD(precision);
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, polytree);
    }

    public static Paths64 InflatePaths(
      Paths64 paths,
      float delta,
      JoinType joinType,
      EndType endType,
      float miterLimit = 2f,
      float arcTolerance = 0f
    )
    {
      ClipperOffset co = new ClipperOffset(miterLimit, arcTolerance);
      co.AddPaths(paths, joinType, endType);
      Paths64 solution = new Paths64();
      co.Execute(delta, solution);
      return solution;
    }

    public static PathsD InflatePaths(
      PathsD paths,
      float delta,
      JoinType joinType,
      EndType endType,
      float miterLimit = 2f,
      int precision = 2,
      float arcTolerance = 0f
    )
    {
      InternalClipper.CheckPrecision(precision);
      float scale = math.exp10(precision);
      Paths64 tmp = ScalePaths64(paths, scale);
      ClipperOffset co = new ClipperOffset(miterLimit, scale * arcTolerance);
      co.AddPaths(tmp, joinType, endType);
      co.Execute(delta * scale, tmp); // reuse 'tmp' to receive (scaled) solution
      return ScalePathsD(tmp, 1 / scale);
    }

    public static Paths64 RectClip(int4 rect, Paths64 paths)
    {
      if (rect.IsEmpty() || paths.Count == 0) return new Paths64();
      RectClip64 rc = new RectClip64(rect);
      return rc.Execute(paths);
    }

    public static Paths64 RectClip(int4 rect, Path64 path)
    {
      if (rect.IsEmpty() || path.Count == 0) return new Paths64();
      Paths64 tmp = new Paths64 { path };
      return RectClip(rect, tmp);
    }
    
    public static PathsD RectClip(float4 rect, PathsD paths, int precision = 2)
    {
      InternalClipper.CheckPrecision(precision);
      if (rect.IsEmpty() || paths.Count == 0) return new PathsD();
      float scale = math.exp10(precision);
      int4 r = ScaleRect(rect, scale);
      Paths64 tmpPath = ScalePaths64(paths, scale);
      RectClip64 rc = new RectClip64(r);
      tmpPath = rc.Execute(tmpPath);
      return ScalePathsD(tmpPath, 1 / scale);
    }

    public static PathsD RectClip(float4 rect, PathD path, int precision = 2)
    {
      if (rect.IsEmpty() || path.Count == 0) return new PathsD();
      PathsD tmp = new PathsD { path };
      return RectClip(rect, tmp, precision);
    }
    public static Paths64 RectClipLines(int4 rect, Paths64 paths)
    {
      if (rect.IsEmpty() || paths.Count == 0) return new Paths64();
      RectClipLines64 rc = new RectClipLines64(rect);
      return rc.Execute(paths);
    }

    public static Paths64 RectClipLines(int4 rect, Path64 path)
    {
      if (rect.IsEmpty() || path.Count == 0) return new Paths64();
      Paths64 tmp = new Paths64 { path };
      return RectClipLines(rect, tmp);
    }

    public static PathsD RectClipLines(float4 rect, 
      PathsD paths, int precision = 2)
    {
      InternalClipper.CheckPrecision(precision);
      if (rect.IsEmpty() || paths.Count == 0) return new PathsD();
      float scale = math.exp10(precision);
      int4 r = ScaleRect(rect, scale);
      Paths64 tmpPath = ScalePaths64(paths, scale);
      RectClipLines64 rc = new RectClipLines64(r);
      tmpPath = rc.Execute(tmpPath);
      return ScalePathsD(tmpPath, 1 / scale);
    }
    public static PathsD RectClipLines(float4 rect, PathD path, int precision = 2)
    {
      if (rect.IsEmpty() || path.Count == 0) return new PathsD();
      PathsD tmp = new PathsD { path };
      return RectClipLines(rect, tmp, precision);
    }
    public static Paths64 MinkowskiSum(Path64 pattern, Path64 path, bool isClosed)
    {
      return Minkowski.Sum(pattern, path, isClosed);
    }

    public static PathsD MinkowskiSum(PathD pattern, PathD path, bool isClosed)
    {
      return Minkowski.Sum(pattern, path, isClosed);
    }

    public static Paths64 MinkowskiDiff(Path64 pattern, Path64 path, bool isClosed)
    {
      return Minkowski.Diff(pattern, path, isClosed);
    }

    public static PathsD MinkowskiDiff(PathD pattern, PathD path, bool isClosed)
    {
      return Minkowski.Diff(pattern, path, isClosed);
    }

    public static float Area(Path64 path)
    {
      // https://en.wikipedia.org/wiki/Shoelace_formula
      float a = 0f;
      if (path.Count < 3)
        return 0f;

      int2 prevPt = path[^1];
      foreach (int2 pt in path)
      {
        a += (prevPt.y + pt.y) * (prevPt.x - pt.x);
        prevPt = pt;
      }
      return a * 0.5f;
    }

    public static double Area(Paths64 paths)
    {
      double a = 0.0;
      foreach (Path64 path in paths)
        a += Area(path);
      return a;
    }

    public static double Area(PathD path)
    {
      double a = 0.0;
      int cnt = path.Count;
      if (cnt < 3) return 0.0;
      float2 prevPt = path[cnt - 1];
      foreach (float2 pt in path)
      {
        a += (prevPt.y + pt.y) * (prevPt.x - pt.x);
        prevPt = pt;
      }
      return a * 0.5;
    }

    public static double Area(PathsD paths)
    {
      double a = 0.0;
      foreach (PathD path in paths)
        a += Area(path);
      return a;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPositive(Path64 poly)
    {
      return Area(poly) >= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPositive(PathD poly)
    {
      return Area(poly) >= 0;
    }

    public static string Path64ToString(Path64 path)
    {
      string result = "";
      foreach (int2 pt in path)
        result += pt.ToString();
      return result + '\n';
    }
    public static string Paths64ToString(Paths64 paths)
    {
      string result = "";
      foreach (Path64 path in paths)
        result += Path64ToString(path);
      return result;
    }
    public static string PathDToString(PathD path)
    {
      string result = "";
      foreach (float2 pt in path)
        result += pt.ToString();
      return result + '\n';
    }
    public static string PathsDToString(PathsD paths)
    {
      string result = "";
      foreach (PathD path in paths)
        result += PathDToString(path);
      return result;
    }
    public static Path64 OffsetPath(Path64 path, int dx, int dy)
    {
      Path64 result = new Path64(path.Count);
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

    public static Path64 ScalePath(Path64 path, float scale)
    {
      if ((scale - 1).IsAlmostZero()) return path;
      Path64 result = new Path64(path.Count);
      foreach (int2 pt in path)
        result.Add((int2)((float2)pt * scale));
      return result;
    }

    public static Paths64 ScalePaths(Paths64 paths, float scale)
    {
      if ((scale - 1).IsAlmostZero()) return paths;
      Paths64 result = new Paths64(paths.Count);
      foreach (Path64 path in paths)
        result.Add(ScalePath(path, scale));
      return result;
    }

    public static PathD ScalePath(PathD path, float scale)
    {
      if ((scale - 1).IsAlmostZero()) return path;
      PathD result = new PathD(path.Count);
      foreach (float2 pt in path)
        result.Add(pt * scale);
      return result;
    }

    public static PathsD ScalePaths(PathsD paths, float scale)
    {
      if ((scale - 1).IsAlmostZero()) return paths;
      PathsD result = new PathsD(paths.Count);
      foreach (PathD path in paths)
        result.Add(ScalePath(path, scale));
      return result;
    }

    // Unlike ScalePath, both ScalePath64 & ScalePathD also involve type conversion
    public static Path64 ScalePath64(PathD path, float scale)
    {
      int cnt = path.Count;
      Path64 res = new Path64(cnt);
      foreach (float2 pt in path)
        res.Add((int2)(pt * scale));
      return res;
    }

    public static Paths64 ScalePaths64(PathsD paths, float scale)
    {
      int cnt = paths.Count;
      Paths64 res = new Paths64(cnt);
      foreach (PathD path in paths)
        res.Add(ScalePath64(path, scale));
      return res;
    }

    public static PathD ScalePathD(Path64 path, float scale)
    {
      int cnt = path.Count;
      PathD res = new PathD(cnt);
      foreach (int2 pt in path)
        res.Add((float2)pt * scale);
      return res;
    }

    public static PathsD ScalePathsD(Paths64 paths, float scale)
    {
      int cnt = paths.Count;
      PathsD res = new PathsD(cnt);
      foreach (Path64 path in paths)
        res.Add(ScalePathD(path, scale));
      return res;
    }

    // The static functions Path64 and PathD convert path types without scaling
    public static Path64 Path64(PathD path)
    {
      Path64 result = new Path64(path.Count);
      foreach (float2 pt in path)
        result.Add(new int2(pt));
      return result;
    }

    public static Paths64 Paths64(PathsD paths)
    {
      Paths64 result = new Paths64(paths.Count);
      foreach (PathD path in paths)
        result.Add(Path64(path));
      return result;
    }

    public static PathsD PathsD(Paths64 paths)
    {
      PathsD result = new PathsD(paths.Count);
      foreach (Path64 path in paths)
        result.Add(PathD(path));
      return result;
    }

    public static PathD PathD(Path64 path)
    {
      PathD result = new PathD(path.Count);
      foreach (int2 pt in path)
        result.Add(new float2(pt));
      return result;
    }

    public static Path64 TranslatePath(Path64 path, int dx, int dy)
    {
      Path64 result = new Path64(path.Count);
      foreach (int2 pt in path)
        result.Add(new int2(pt.x + dx, pt.y + dy));
      return result;
    }

    public static Paths64 TranslatePaths(Paths64 paths, int dx, int dy)
    {
      Paths64 result = new Paths64(paths.Count);
      foreach (Path64 path in paths)
        result.Add(OffsetPath(path, dx, dy));
      return result;
    }

    public static PathD TranslatePath(PathD path, float dx, float dy)
    {
      PathD result = new PathD(path.Count);
      foreach (float2 pt in path)
        result.Add(new float2(pt.x + dx, pt.y + dy));
      return result;
    }

    public static PathsD TranslatePaths(PathsD paths, float dx, float dy)
    {
      PathsD result = new PathsD(paths.Count);
      foreach (PathD path in paths)
        result.Add(TranslatePath(path, dx, dy));
      return result;
    }

    public static Path64 ReversePath(Path64 path)
    {
      Path64 result = new Path64(path);
      result.Reverse();
      return result;
    }

    public static PathD ReversePath(PathD path)
    {
      PathD result = new PathD(path);
      result.Reverse();
      return result;
    }

    public static Paths64 ReversePaths(Paths64 paths)
    {
      Paths64 result = new Paths64(paths.Count);
      foreach (Path64 t in paths)
        result.Add(ReversePath(t));

      return result;
    }

    public static PathsD ReversePaths(PathsD paths)
    {
      PathsD result = new PathsD(paths.Count);
      foreach (PathD path in paths)
        result.Add(ReversePath(path));
      return result;
    }

    public static int4 GetBounds(Path64 path)
    {
      int4 result = InvalidRectI;
      foreach (int2 pt in path)
      {
        if (pt.x < result.x) result.x = pt.x;
        if (pt.x > result.z) result.z = pt.x;
        if (pt.y < result.z) result.z = pt.y;
        if (pt.y > result.w) result.w = pt.y;
      }
      return result.x == long.MaxValue ? new int4() : result;
    }

    public static int4 GetBounds(Paths64 paths)
    {
      int4 result = InvalidRectI;
      foreach (Path64 path in paths)
        foreach (int2 pt in path)
        {
          if (pt.x < result.x) result.x = pt.x;
          if (pt.x > result.z) result.z = pt.x;
          if (pt.y < result.z) result.z = pt.y;
          if (pt.y > result.w) result.w = pt.y;
        }
      return result.x == long.MaxValue ? new int4() : result;
    }

    public static float4 GetBounds(PathD path)
    {
      float4 result = InvalidRectF;
      foreach (float2 pt in path)
      {
        if (pt.x < result.x) result.x = pt.x;
        if (pt.x > result.z) result.z = pt.x;
        if (pt.y < result.y) result.y = pt.y;
        if (pt.y > result.w) result.w = pt.y;
      }
      return Math.Abs(result.x - double.MaxValue) < InternalClipper.floatingPointTolerance ? new float4() : result;
    }

    public static float4 GetBounds(PathsD paths)
    {
      float4 result = InvalidRectF;
      foreach (PathD path in paths)
        foreach (float2 pt in path)
        {
          if (pt.x < result.x) result.x = pt.x;
          if (pt.x > result.z) result.z = pt.x;
          if (pt.y < result.y) result.y = pt.y;
          if (pt.y > result.w) result.w = pt.y;
        }
      return Math.Abs(result.x - double.MaxValue) < InternalClipper.floatingPointTolerance ? new float4() : result;
    }

    public static Path64 MakePath(int[] arr)
    {
      int len = arr.Length / 2;
      Path64 p = new Path64(len);
      for (int i = 0; i < len; i++)
        p.Add(new int2(arr[i * 2], arr[i * 2 + 1]));
      return p;
    }

    public static PathD MakePath(float[] arr)
    {
      int len = arr.Length / 2;
      PathD p = new PathD(len);
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

    public static PathD StripNearDuplicates(PathD path,
        double minEdgeLenSqrd, bool isClosedPath)
    {
      int cnt = path.Count;
      PathD result = new PathD(cnt);
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

    public static Path64 StripDuplicates(Path64 path, bool isClosedPath)
    {
      int cnt = path.Count;
      Path64 result = new Path64(cnt);
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
    private static void AddPolyNodeToPaths(PolyPath64 polyPath, Paths64 paths)
    {
      if (polyPath.Polygon!.Count > 0)
        paths.Add(polyPath.Polygon);
      for (int i = 0; i < polyPath.Count; i++)
        AddPolyNodeToPaths((PolyPath64) polyPath._childs[i], paths);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Paths64 PolyTreeToPaths64(PolyTree64 polyTree)
    {
      Paths64 result = new Paths64();
      for (int i = 0; i < polyTree.Count; i++)
        AddPolyNodeToPaths((PolyPath64) polyTree._childs[i], result);
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddPolyNodeToPathsD(PolyPathD polyPath, PathsD paths)
    {
      if (polyPath.Polygon!.Count > 0)
        paths.Add(polyPath.Polygon);
      for (int i = 0; i < polyPath.Count; i++)
        AddPolyNodeToPathsD((PolyPathD) polyPath._childs[i], paths);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PathsD PolyTreeToPathsD(PolyTreeD polyTree)
    {
      PathsD result = new PathsD();
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

    internal static void RDP(Path64 path, int begin, int end, double epsSqrd, List<bool> flags)
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

    public static Path64 RamerDouglasPeucker(Path64 path, float epsilon)
    {
      int len = path.Count;
      if (len < 5) return path;
      List<bool> flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
      RDP(path, 0, len - 1, Sqr(epsilon), flags);
      Path64 result = new Path64(len);
      for (int i = 0; i < len; ++i)
        if (flags[i]) result.Add(path[i]);
      return result;
    }

    public static Paths64 RamerDouglasPeucker(Paths64 paths, float epsilon)
    {
      Paths64 result = new Paths64(paths.Count);
      foreach (Path64 path in paths)
        result.Add(RamerDouglasPeucker(path, epsilon));
      return result;
    }

    internal static void RDP(PathD path, int begin, int end, float epsSqrd, List<bool> flags)
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

    public static PathD RamerDouglasPeucker(PathD path, float epsilon)
    {
      int len = path.Count;
      if (len < 5) return path;
      List<bool> flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
      RDP(path, 0, len - 1, Sqr(epsilon), flags);
      PathD result = new PathD(len);
      for (int i = 0; i < len; ++i)
        if (flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsD RamerDouglasPeucker(PathsD paths, float epsilon)
    {
      PathsD result = new PathsD(paths.Count);
      foreach (PathD path in paths)
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

      public static Path64 SimplifyPath(
        Path64 path,
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
      Path64 result = new Path64(len);
      for (int i = 0; i < len; i++)
        if (!flags[i]) result.Add(path[i]);
      return result;
    }

    public static Paths64 SimplifyPaths(
      Paths64 paths,
      float epsilon,
      bool isClosedPaths = true
    )
    {
      Paths64 result = new Paths64(paths.Count);
      foreach (Path64 path in paths)
        result.Add(SimplifyPath(path, epsilon, isClosedPaths));
      return result;
    }

    public static PathD SimplifyPath(
      PathD path,
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
      PathD result = new PathD(len);
      for (int i = 0; i < len; i++)
        if (!flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsD SimplifyPaths(
      PathsD paths,
      float epsilon,
      bool isClosedPath = true
    )
    {
      PathsD result = new PathsD(paths.Count);
      foreach (PathD path in paths)
        result.Add(SimplifyPath(path, epsilon, isClosedPath));
      return result;
    }

    public static Path64 TrimCollinear(Path64 path, bool isOpen = false)
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
          return new Path64();
        return path;
      }

      Path64 result = new Path64(len - i);
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

    public static PathD TrimCollinear(PathD path, int precision, bool isOpen = false)
    {
      InternalClipper.CheckPrecision(precision);
      float scale = math.exp10(precision);
      Path64 p = ScalePath64(path, scale);
      p = TrimCollinear(p, isOpen);
      return ScalePathD(p, 1 / scale);
    }

    public static PointInPolygonResult PointInPolygon(int2 pt, Path64 polygon)
    {
      return InternalClipper.PointInPolygon(pt, polygon);
    }

    public static PointInPolygonResult PointInPolygon(float2 pt, 
      PathD polygon, int precision = 2)
    {
      InternalClipper.CheckPrecision(precision);
      float scale = math.exp10(precision);
      int2 p = (int2)(pt * scale);
      Path64 path = ScalePath64(polygon, scale);
      return InternalClipper.PointInPolygon(p, path);
    }

    public static Path64 Ellipse(
      int2 center,
      float radiusX,
      float radiusY = 0,
      int steps = 0
    )
    {
      if (radiusX <= 0) return new Path64();
      if (radiusY <= 0) radiusY = radiusX;
      if (steps <= 2)
        steps = (int) math.ceil(math.PI * math.sqrt((radiusX + radiusY) / 2));

      float2 radius = new float2(radiusX, radiusY);

      math.sincos(math.TAU / steps, out float si, out float co);
      float2 d = new float2(co, si);
      Path64 result = new Path64(steps) { new int2((int)(center.x + radiusX), center.y) };
      for (int i = 1; i < steps; ++i)
      {
        result.Add((int2)math.round(center + radius * d));
        float x = d.x * co - d.y * si;
        d.y = d.y * co + d.x * si;
        d.x = x;
      }
      return result;
    }

    public static PathD Ellipse(
      float2 center,
      float radiusX,
      float radiusY = 0,
      int steps = 0
    )
    {
      if (radiusX <= 0) return new PathD();
      if (radiusY <= 0) radiusY = radiusX;
      if (steps <= 2)
        steps = (int) math.ceil(math.PI * math.sqrt((radiusX + radiusY) / 2));

      float2 radius = new float2(radiusX, radiusY);

      math.sincos(math.TAU / steps, out float si, out float co);
      float2 d = new float2(co, si);
      PathD result = new PathD(steps) { new float2(center.x + radiusX, center.y) };
      for (int i = 1; i < steps; ++i)
      {
        result.Add(center + radius * d);
        float x = d.x * co - d.y * si;
        d.y = d.y * co + d.x * si;
        d.x = x;
      }
      return result;
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

    public static TriangulateResult Triangulate(Paths64 pp, out Paths64 solution, bool useDelaunay = true)
    {
      Delaunay d = new Delaunay(useDelaunay);
      return d.Execute(pp, out solution);
    }

    public static TriangulateResult Triangulate(PathsD pp, int decPlaces, out PathsD solution, bool useDelaunay = true)
    {
      float scale = decPlaces <= 0 ? 1f : math.exp10(math.min(decPlaces, 8));

      Paths64 pp64 = ScalePaths64(pp, scale);

      Delaunay d = new Delaunay(useDelaunay);
      TriangulateResult result = d.Execute(pp64, out Paths64 sol64);
      if (result == TriangulateResult.success)
        solution = ScalePathsD(sol64, 1f / scale);
      else
        solution = new PathsD();
      return result;
    }

  } // Clipper
} // namespace