/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  12 December 2025                                                *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  Core structures and functions for the Clipper Library           *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Clipper2Lib
{

  public class Path64 : List<int2>
  {
    public Path64() : base() { }
    public Path64(int capacity = 0) : base(capacity) { }
    public Path64(IEnumerable<int2> path) : base(path) { }
    public override string ToString()
    {
      return string.Join(", ", this);
    }
  }

  public class Paths64 : List<Path64>
  {
    public Paths64() : base() { }
    public Paths64(int capacity = 0) : base(capacity) { }
    public Paths64(IEnumerable<Path64> paths) : base(paths) { }
    public override string ToString()
    {
      return string.Join(Environment.NewLine, this);
    }
  }

  public class PathD : List<float2>
  {
    public PathD() : base() { }
    public PathD(int capacity = 0) : base(capacity) { }
    public PathD(IEnumerable<float2> path) : base(path) { }
    public override string ToString()
      => string.Join(", ", this);
  }

  public class PathsD : List<PathD>
  {
    public PathsD() : base() { }
    public PathsD(int capacity = 0) : base(capacity) { }
    public PathsD(IEnumerable<PathD> paths) : base(paths) { }
    public override string ToString()
      => string.Join(Environment.NewLine, this);
  }

  // Note: all clipping operations except for Difference are commutative.
  public enum ClipType
  {
    NoClip,
    Intersection,
    Union,
    Difference,
    Xor
  }

  public enum PathType
  {
    Subject,
    Clip
  }

  // By far the most widely used filling rules for polygons are EvenOdd
  // and NonZero, sometimes called Alternate and Winding respectively.
  // https://en.wikipedia.org/wiki/Nonzero-rule
  public enum FillRule
  {
    EvenOdd,
    NonZero,
    Positive,
    Negative
  }

  internal static class InternalClipper
  {
    public const long MaxInt64 = 9223372036854775807;
    public const long MaxCoord = MaxInt64 / 4;
    public const double max_coord = MaxCoord;
    public const double min_coord = -MaxCoord;
    public const long Invalid64 = MaxInt64;

    internal const double floatingPointTolerance = 1E-12;

    public static int CrossProductSign(int2 pt1, int2 pt2, int2 pt3)
    {
      int a = pt2.x - pt1.x;
      int b = pt3.y - pt2.y;
      int c = pt2.y - pt1.y;
      int d = pt3.x - pt2.x;
      UInt128Struct ab = MultiplyUInt64((ulong) Math.Abs(a), (ulong) Math.Abs(b));
      UInt128Struct cd = MultiplyUInt64((ulong) Math.Abs(c), (ulong) Math.Abs(d));
      int signAB = math.sign(a) * math.sign(b);
      int signCD = math.sign(c) * math.sign(d);

      if (signAB == signCD)
      {
        int result;
        if (ab.hi64 == cd.hi64)
        {
          if (ab.lo64 == cd.lo64) return 0;
          result = (ab.lo64 > cd.lo64) ? 1 : -1;
        }
        else result = (ab.hi64 > cd.hi64) ? 1 : -1;
        return (signAB > 0) ? result : -result;
      }
      return (signAB > signCD) ? 1 : -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float PrecisionToScale(int precision)
      => math.exp10(math.clamp(precision, -8, 8));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int TriSign(long x) // returns 0, 1 or -1
    {
      return (x < 0) ? -1 : (x > 0) ? 1 : 0;
    }

    public struct UInt128Struct
    {
      public ulong lo64;
      public ulong hi64;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static UInt128Struct MultiplyUInt64(ulong a, ulong b) // #834,#835
    {
      ulong x1 = (a & 0xFFFFFFFF) * (b & 0xFFFFFFFF);
      ulong x2 = (a >> 32) * (b & 0xFFFFFFFF) + (x1 >> 32);
      ulong x3 = (a & 0xFFFFFFFF) * (b >> 32) + (x2 & 0xFFFFFFFF);
      UInt128Struct result;
      result.lo64 = (x3 & 0xFFFFFFFF) << 32 | (x1 & 0xFFFFFFFF);
      result.hi64 = (a >> 32) * (b >> 32) + (x2 >> 32) + (x3 >> 32);
      return result;
    }

    // returns true if (and only if) a * b == c * d
    internal static bool ProductsAreEqual(int a, int b, int c, int d)
    {
      // nb: unsigned values will be needed for CalcOverflowCarry()
      ulong absA = (ulong)math.abs(a);
      ulong absB = (ulong)math.abs(b);
      ulong absC = (ulong)math.abs(c);
      ulong absD = (ulong)math.abs(d);

      ulong mul_ab = absA * absB;
      ulong mul_cd = absC * absD;

      // nb: it's important to differentiate 0 values here from other values
      int sign_ab = math.sign(a) * math.sign(b);
      int sign_cd = math.sign(c) * math.sign(d);

      return mul_ab == mul_cd && sign_ab == sign_cd;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsCollinear(int2 pt1, int2 sharedPt, int2 pt2)
    {
      int a = sharedPt.x - pt1.x;
      int b = pt2.y - sharedPt.y;
      int c = sharedPt.y - pt1.y;
      int d = pt2.x - sharedPt.x;
      // When checking for collinearity with very large coordinate values
      // then ProductsAreEqual is more accurate than using CrossProduct.
      return ProductsAreEqual(a, b, c, d);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float DotProduct(int2 pt1, int2 pt2, int2 pt3)
    {
      float2 a = pt2 - pt1;
      float2 b = pt3 - pt2;
      return math.dot(a, b);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float CrossProduct(float2 vec1, float2 vec2)
    {
      return (vec1.y * vec2.x - vec2.y * vec1.x);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static long CheckCastInt64(double val)
    {
      if ((val >= max_coord) || (val <= min_coord)) return Invalid64;
      return (long)Math.Round(val, MidpointRounding.AwayFromZero);
    }

    // GetLineIntersectPt - a 'true' result is non-parallel. The 'ip' will also
    // be constrained to seg1. However, it's possible that 'ip' won't be inside
    // seg2, even when 'ip' hasn't been constrained (ie 'ip' is inside seg1).

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool GetLineIntersectPt(int2 ln1a, int2 ln1b, int2 ln2a, int2 ln2b, out int2 ip)
    {
      float2 d1 = ln1b - ln1a;
      float2 d2 = ln2b - ln2a;
      float det = d1.y * d2.x - d2.y * d1.x;
      if (det == 0f)
      {
        ip = default;
        return false;
      }

      float2 deltaA = ln1a - ln2a;

      float t = (deltaA.x * d2.y - deltaA.y * d2.x) / det;
      if (t <= 0.0) ip = ln1a;
      else if (t >= 1.0) ip = ln1b;
      else
      {
        // avoid using constructor (and rounding too) as they affect performance //664
        ip = (int2)((float2)ln1a + (d1 * t));
      }
      return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool GetLineIntersectPt(float2 ln1a, float2 ln1b, float2 ln2a, float2 ln2b, out float2 ip)
    {
      float2 d1 = ln1b - ln1a;
      float2 d2 = ln2b - ln2a;
      float det = d1.y * d2.x - d2.y * d1.x;
      if (det == 0.0)
      {
        ip = new float2();
        return false;
      }

      float2 deltaA = ln1a - ln2a;

      float t = (deltaA.x * d2.y - deltaA.y * d2.x) / det;
      if (t <= 0.0) ip = ln1a;
      else if (t >= 1.0) ip = ln1b;
      else
      {
        // avoid using constructor (and rounding too) as they affect performance //664
        ip = ln1a + d1 * t;
      }
      return true;
    }

    internal static bool SegsIntersect(int2 seg1a,
      int2 seg1b, int2 seg2a, int2 seg2b, bool inclusive = false)
    {
      double dy1 = (seg1b.y - seg1a.y);
      double dx1 = (seg1b.x - seg1a.x);
      double dy2 = (seg2b.y - seg2a.y);
      double dx2 = (seg2b.x - seg2a.x);
      double cp = dy1 * dx2 - dy2 * dx1;
      if (cp == 0) return false; // ie parallel segments

      if (inclusive)
      {
        //result **includes** segments that touch at an end point
        double t = ((seg1a.x - seg2a.x) * dy2 - (seg1a.y - seg2a.y) * dx2);
        if (t == 0) return true;
        if (t > 0)
        {
          if (cp < 0 || t > cp) return false;
        }
        else if (cp > 0 || t < cp) return false; // false when t more neg. than cp

        t = ((seg1a.x - seg2a.x) * dy1 - (seg1a.y - seg2a.y) * dx1);
        if (t == 0) return true;
        if (t > 0) return (cp > 0 && t <= cp);
        else return (cp < 0 && t >= cp);        // true when t less neg. than cp
      }
      else
      {
        //result **excludes** segments that touch at an end point
        double t = ((seg1a.x - seg2a.x) * dy2 - (seg1a.y - seg2a.y) * dx2);
        if (t == 0) return false;
        if (t > 0)
        {
          if (cp < 0 || t >= cp) return false;
        }
        else if (cp > 0 || t <= cp) return false; // false when t more neg. than cp

        t = ((seg1a.x - seg2a.x) * dy1 - (seg1a.y - seg2a.y) * dx1);
        if (t == 0) return false;
        if (t > 0) return (cp > 0 && t < cp);
        else return (cp < 0 && t > cp); // true when t less neg. than cp
      }
    }

    public static int4 GetBounds(Path64 path)
    {
      if (path.Count == 0) return new int4();
      int4 result = Clipper.InvalidRectI;
      foreach (int2 pt in path)
      {
        if (pt.x < result.x) result.x = pt.x;
        if (pt.x > result.z) result.z = pt.x;
        if (pt.y < result.y) result.y = pt.y;
        if (pt.y > result.w) result.w = pt.y;
      }
      return result;
    }

    public static int2 GetClosestPtOnSegment(int2 offPt,
    int2 seg1, int2 seg2)
    {
      if (seg1.x == seg2.x && seg1.y == seg2.y) return seg1;
      float dx = (seg2.x - seg1.x);
      float dy = (seg2.y - seg1.y);
      float q = ((offPt.x - seg1.x) * dx +
        (offPt.y - seg1.y) * dy) / ((dx*dx) + (dy*dy));
      if (q < 0) q = 0; else if (q > 1) q = 1;
      return new int2(
        // use MidpointRounding.ToEven in order to explicitly match the nearbyint behaviour on the C++ side
        seg1.x + (int)math.round(q * dx),
        seg1.y + (int)math.round(q * dy)
      );
    }

    public static PointInPolygonResult PointInPolygon(int2 pt, Path64 polygon)
    {
      int len = polygon.Count, start = 0;
      if (len < 3) return PointInPolygonResult.IsOutside;

      while (start < len && polygon[start].y == pt.y) start++;
      if (start == len) return PointInPolygonResult.IsOutside;

      bool isAbove = polygon[start].y < pt.y, startingAbove = isAbove;
      int val = 0, i = start + 1, end = len;
      while (true)
      {
        if (i == end)
        {
          if (end == 0 || start == 0) break;
          end = start;
          i = 0;
        }

        if (isAbove)
        {
          while (i < end && polygon[i].y < pt.y) i++;
        }
        else
        {
          while (i < end && polygon[i].y > pt.y) i++;
        }

        if (i == end) continue;

        int2 curr = polygon[i], prev;
        if (i > 0) prev = polygon[i - 1];
        else prev = polygon[len - 1];

        if (curr.y == pt.y)
        {
          if (curr.x == pt.x || (curr.y == prev.y &&
            ((pt.x < prev.x) != (pt.x < curr.x))))
            return PointInPolygonResult.IsOn;
          i++;
          if (i == start) break;
          continue;
        }

        if (pt.x < curr.x && pt.x < prev.x)
        {
          // we're only interested in edges crossing on the left
        }
        else if (pt.x > prev.x && pt.x > curr.x)
        {
          val = 1 - val; // toggle val
        }
        else
        {
          int cps2 = CrossProductSign(prev, curr, pt);
          if (cps2 == 0) return PointInPolygonResult.IsOn;
          if ((cps2 < 0) == isAbove) val = 1 - val;
        }
        isAbove = !isAbove;
        i++;
      }

      if (isAbove == startingAbove) return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
      if (i == len) i = 0;
      int cps = (i == 0) ?
        CrossProductSign(polygon[len - 1], polygon[0], pt) :
        CrossProductSign(polygon[i - 1], polygon[i], pt);

      if (cps == 0) return PointInPolygonResult.IsOn;
      if ((cps < 0) == isAbove) val = 1 - val;
      return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
    }

    public static bool Path2ContainsPath1(Path64 path1, Path64 path2)
    {
      // we need to make some accommodation for rounding errors
      // so we won't jump if the first vertex is found outside
      PointInPolygonResult pip = PointInPolygonResult.IsOn;
      foreach (int2 pt in path1)
      {
        switch (PointInPolygon(pt, path2))
        {
          case PointInPolygonResult.IsOutside:
            if (pip == PointInPolygonResult.IsOutside) return false;
            pip = PointInPolygonResult.IsOutside;
            break;
          case PointInPolygonResult.IsInside:
            if (pip == PointInPolygonResult.IsInside) return true;
            pip = PointInPolygonResult.IsInside;
            break;
          default: break;
        }
      }
      // since path1's location is still equivocal, check its midpoint
      int2 mp = GetBounds(path1).MidPoint();
      return InternalClipper.PointInPolygon(mp, path2) != PointInPolygonResult.IsOutside;
    }


  } // InternalClipper

} // namespace