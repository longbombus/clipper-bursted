using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.Collections;

/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  11 October 2025                                                 *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  Path Offset (Inflate/Shrink)                                    *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

namespace Clipper
{
  public enum JoinType
  {
    Miter,
    Square,
    Bevel,
    Round
  }

  public enum EndType
  {
    Polygon,
    Joined,
    Butt,
    Square,
    Round
  }

  public class ClipperOffset
  {

    private class Group
    {
      internal NativeSlicedList<int2> inPaths;
      internal JoinType joinType;
      internal EndType endType;
      internal bool pathsReversed;
      internal int lowestPathIdx;

      public Group(NativeSlicedList<int2> paths, JoinType joinType, EndType endType = EndType.Polygon, Allocator allocatorFields = Allocator.Temp)
      {
        this.joinType = joinType;
        this.endType = endType;

        bool isJoined = ((endType == EndType.Polygon) || (endType == EndType.Joined));
        inPaths = new NativeSlicedList<int2>(allocatorFields);
        // copy and strip duplicates per path
        for (int si = 0; si < paths.SlicesCount; si++)
        {
          var slice = paths[si];
          PathI tmp = new PathI(slice.Length);
          for (int i = 0; i < slice.Length; i++) tmp.Add(slice[i]);
          tmp = Clipper.StripDuplicates(tmp, isJoined);
          // append tmp to inPaths
          foreach (int2 pt in tmp) inPaths.AddLastSliceItem(pt);
          inPaths.FinishSlice();
        }

        if (endType == EndType.Polygon)
        {
          bool isNegArea;
          GetLowestPathInfo(convertPathsI(inPaths), out lowestPathIdx, out isNegArea);
          pathsReversed = (lowestPathIdx >= 0) && isNegArea;
        }
        else
        {
          lowestPathIdx = -1;
          pathsReversed = false;
        }
      }
    }

    private const double Tolerance = 1.0E-12;

    // Clipper2 approximates arcs by using series of relatively short straight
    //line segments. And logically, shorter line segments will produce better arc
    // approximations. But very short segments can degrade performance, usually
    // with little or no discernable improvement in curve quality. Very short
    // segments can even detract from curve quality, due to the effects of integer
    // rounding. Since there isn't an optimal number of line segments for any given
    // arc radius (that perfectly balances curve approximation with performance),
    // arc tolerance is user defined. Nevertheless, when the user doesn't define
    // an arc tolerance (ie leaves alone the 0 default value), the calculated
    // default arc tolerance (offset_radius / 500) generally produces good (smooth)
    // arc approximations without producing excessively small segment lengths.
    // See also: https://www.angusj.com/clipper2/Docs/Trigonometry.htm
    private const float arc_const = 0.002f; // <-- 1/500

    private readonly List<Group> _groupList = new List<Group>();
    private PathI pathOut = new PathI();
    private readonly PathF _normals = new PathF();
    private PathsI _solution = new PathsI();
    private PolyTree64? _solutionTree;

    private readonly Allocator _allocatorFields;
    private readonly Allocator _allocatorTemp;

    private float _groupDelta; //*0.5 for open paths; *-1.0 for negative areas
    private float _delta;
    private float _mitLimSqr;
    private float _stepsPerRad;
    private float _stepSin;
    private float _stepCos;
    private JoinType _joinType;
    private EndType _endType;
    public float ArcTolerance { get; set; }
    public bool MergeGroups { get; set; }
    public float MiterLimit { get; set; }
    public bool PreserveCollinear { get; set; }
    public bool ReverseSolution { get; set; }

    public delegate float DeltaCallback64(PathI path, PathF path_norms, int currPt, int prevPt);
    public DeltaCallback64 DeltaCallback { get; set; }

    public ClipperOffset(Allocator allocatorFields, Allocator allocatorTemp,
      float miterLimit = 2f,
      float arcTolerance = 0f,
      bool preserveCollinear = false,
      bool reverseSolution = false
    )
    {
      _allocatorFields = allocatorFields;
      _allocatorTemp = allocatorTemp;
      MiterLimit = miterLimit;
      ArcTolerance = arcTolerance;
      MergeGroups = true;
      PreserveCollinear = preserveCollinear;
      ReverseSolution = reverseSolution;
    }

    public void Dispose()
    {
      // dispose groups inPaths
      foreach (var g in _groupList)
      {
        g.inPaths.Dispose();
      }
      _solution = new PathsI();
    }

    public void Clear()
    {
      // dispose any existing group inPaths
      foreach (var g in _groupList)
        g.inPaths.Dispose();
      _groupList.Clear();
    }

    public void AddPath(NativeArray<int2> path, JoinType joinType, EndType endType)
    {
      int cnt = path.Length;
      if (cnt == 0) return;
      NativeSlicedList<int2> sl = new NativeSlicedList<int2>(_allocatorTemp);
      for (int i = 0; i < cnt; i++) sl.AddLastSliceItem(path[i]);
      sl.FinishSlice();
      _groupList.Add(new Group(sl, joinType, endType, _allocatorFields));
      // temp sl disposed inside Group copy? Dispose local sl
      sl.Dispose();
    }

    public void AddPaths(NativeSlicedList<int2> paths, JoinType joinType, EndType endType)
    {
      if (paths.SlicesCount == 0) return;
      _groupList.Add(new Group(paths, joinType, endType, _allocatorFields));
    }

    // helper to convert SlicedList<int2> to PathsI for existing routines
    private static PathsI convertPathsI(NativeSlicedList<int2> s)
    {
      PathsI res = new PathsI(s.SlicesCount);
      for (int si = 0; si < s.SlicesCount; si++)
      {
        var slice = s[si];
        PathI p = new PathI(slice.Length);
        for (int i = 0; i < slice.Length; i++) p.Add(slice[i]);
        res.Add(p);
      }
      return res;
    }

    private int CalcSolutionCapacity()
    {
      int result = 0;
      foreach (Group g in _groupList)
        result += (g.endType == EndType.Joined) ? g.inPaths.SlicesCount * 2 : g.inPaths.SlicesCount;
      return result;
    }

    internal bool CheckPathsReversed()
    {
      bool result = false;
      foreach (Group g in _groupList)
        if (g.endType == EndType.Polygon)
        {
          result = g.pathsReversed;
          break;
        }
      return result;
    }

    private void ExecuteInternal(float delta)
    {
      if (_groupList.Count == 0) return;
      _solution.EnsureCapacity(CalcSolutionCapacity());

      // make sure the offset delta is significant
      if (Math.Abs(delta) < 0.5)
      {
        foreach (Group group in _groupList)
        {
          PathsI tmp = convertPathsI(group.inPaths);
          foreach (var path in tmp)
            _solution.Add(path);
        }
        return;
      }

      _delta = delta;
      _mitLimSqr = MiterLimit <= 1 ? 2f : 2f / MiterLimit.Sqr();

      foreach (Group group in _groupList)
        DoGroupOffset(group);

      if (_groupList.Count == 0) return;

      bool pathsReversed = CheckPathsReversed();
      FillRule fillRule = pathsReversed ? FillRule.Negative : FillRule.Positive;

      // clean up self-intersections ...
      Clipper64 c = new Clipper64();
      c.PreserveCollinear = PreserveCollinear;
      c.ReverseSolution = ReverseSolution != pathsReversed;

      c.AddSubject(_solution);
      if (_solutionTree != null)
        c.Execute(ClipType.Union, fillRule, _solutionTree);
      else
        c.Execute(ClipType.Union, fillRule, _solution);

    }

    public void Execute(float delta, PathsI solution)
    {
      solution.Clear();
      _solution = solution;
      ExecuteInternal(delta);
    }

    public void Execute(float delta, PolyTree64 solutionTree)
    {
      solutionTree.Clear();
      _solutionTree = solutionTree;
      _solution.Clear();
      ExecuteInternal(delta);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float2 GetUnitNormal(int2 pt1, int2 pt2)
    {
      int2 d = pt1 - pt2;
      if (!math.any(d))
        return new float2();

      float f = math.rsqrt(math.dot(d, d));
      float2 result = (float2)d * f;

      return new float2(result.y, -result.x);
    }

    public void Execute(DeltaCallback64 deltaCallback, PathsI solution)
    {
      DeltaCallback = deltaCallback;
      Execute(1f, solution);
    }    
    
    internal static void GetLowestPathInfo(PathsI paths, out int idx, out bool isNegArea)
    {
      idx = -1;
      isNegArea = false;
      int2 botPt = new int2(int.MaxValue, int.MinValue);
      for (int i = 0; i < paths.Count; ++i)
      {
        float a = float.PositiveInfinity;
        foreach (int2 pt in paths[i])
		    {
          if ((pt.y < botPt.y) ||
            ((pt.y == botPt.y) && (pt.x >= botPt.x))) continue;
          if (float.IsInfinity(a))
          {
            a = Clipper.Area(paths[i]);
            if (a == 0) break; // invalid closed path so break from inner loop
            isNegArea = a < 0;
          }
          idx = i;
          botPt.x = pt.x;
          botPt.y = pt.y;
        }
      }
    }

  [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float2 TranslatePoint(float2 pt, float dx, float dy)
      => new(pt.x + dx, pt.y + dy);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float2 ReflectPoint(float2 pt, float2 pivot)
      => pivot + (pivot - pt);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float2 NormalizeVector(float2 vec)
    {
	    float h = math.length(vec);
	    if (h.IsAlmostZero())
        return new float2(0,0);

      float inverseHypot = math.rcp(h);
	    return vec * inverseHypot;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float2 GetAvgUnitVector(float2 vec1, float2 vec2)
    {
	    return NormalizeVector(new float2(vec1.x + vec2.x, vec1.y + vec2.y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int2 GetPerpendic(int2 pt, float2 norm)
      => (int2)(pt + norm * _groupDelta);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float2 GetPerpendicD(int2 pt, float2 norm)
    {
      return new float2(pt.x + norm.x * _groupDelta,
        pt.y + norm.y * _groupDelta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoBevel(PathI path, int j, int k)
    {
      int2 pt1, pt2;
      if (j == k)
      {
        float absDelta = math.abs(_groupDelta);
        int2 offset = (int2)math.round(_normals[j] * absDelta);

        pt1 = path[j] - offset;
        pt2 = path[j] + offset;
      }
      else
      {
        pt1 = path[j] + (int2)math.round(_normals[k] * _groupDelta);
        pt2 = path[j] + (int2)math.round(_normals[j] * _groupDelta);
      }
      pathOut.Add(pt1);
      pathOut.Add(pt2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoSquare(PathI path, int j, int k)
    {
      float2 vec;
      if (j == k)
      {
        vec = new float2(_normals[j].y, -_normals[j].x);
      }
      else
      {
        vec = GetAvgUnitVector(
          new float2(-_normals[k].y, _normals[k].x),
          new float2(_normals[j].y, -_normals[j].x));
      }

      float absDelta = math.abs(_groupDelta);
      // now offset the original vertex delta units along unit vector
      float2 ptQ = new float2(path[j]);
      ptQ = TranslatePoint(ptQ, absDelta * vec.x, absDelta * vec.y);

      // get perpendicular vertices
      float2 pt1 = TranslatePoint(ptQ, _groupDelta * vec.y, _groupDelta * -vec.x);
      float2 pt2 = TranslatePoint(ptQ, _groupDelta * -vec.y, _groupDelta * vec.x);
      // get 2 vertices along one edge offset
      float2 pt3 = GetPerpendicD(path[k], _normals[k]);

      if (j == k)
      {
        float2 pt4 = new float2(
          pt3.x + vec.x * _groupDelta,
          pt3.y + vec.y * _groupDelta);
        InternalClipper.GetLineIntersectPt(pt1, pt2, pt3, pt4, out float2 pt);
        //get the second intersect point through reflecion
        pathOut.Add(new int2(ReflectPoint(pt, ptQ)));
        pathOut.Add(new int2(pt));
      }
      else
      {
        float2 pt4 = GetPerpendicD(path[j], _normals[k]);
        InternalClipper.GetLineIntersectPt(pt1, pt2, pt3, pt4, out float2 pt);
        pathOut.Add(new int2(pt));
        //get the second intersect point through reflecion
        pathOut.Add(new int2(ReflectPoint(pt, ptQ)));
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoMiter(PathI path, int j, int k, float cosA)
    {
      float q = _groupDelta / (cosA + 1);

      pathOut.Add(path[j] + (int2)math.round((_normals[k] + _normals[j]) * q));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoRound(PathI path, int j, int k, float angle)
    {
      if (DeltaCallback != null)
      {
        // when DeltaCallback is assigned, _groupDelta won't be constant,
        // so we'll need to do the following calculations for *every* vertex.
        float absDelta = math.abs(_groupDelta);
        float arcTol = ArcTolerance > 0.01f ? ArcTolerance : absDelta * arc_const;
        float stepsPer360 = math.PI / math.acos(1 - arcTol / absDelta);
        math.sincos(math.TAU / stepsPer360, out _stepSin, out _stepCos);

        if (_groupDelta < 0.0) _stepSin = -_stepSin;
        _stepsPerRad = stepsPer360 / math.TAU;
      }

      int2 pt = path[j];
      float2 offsetVec = new float2(_normals[k].x * _groupDelta, _normals[k].y * _groupDelta);
      if (j == k)
        offsetVec = -offsetVec;

      pathOut.Add((int2)(pt + offsetVec));

      int steps = (int) Math.Ceiling(_stepsPerRad * Math.Abs(angle));
      for (int i = 1; i < steps; i++) // ie 1 less than steps
      {
        offsetVec = new float2(offsetVec.x * _stepCos - _stepSin * offsetVec.y,
            offsetVec.x * _stepSin + offsetVec.y * _stepCos);

        pathOut.Add((int2)(pt + offsetVec));
      }
      pathOut.Add(GetPerpendic(pt, _normals[j]));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void BuildNormals(PathI path)
    {
      int cnt = path.Count;
      _normals.Clear();
      if (cnt == 0) return;
      _normals.EnsureCapacity(cnt);
      for (int i = 0; i < cnt - 1; i++)
        _normals.Add(GetUnitNormal(path[i], path[i + 1]));
      _normals.Add(GetUnitNormal(path[cnt - 1], path[0]));
    }

    private void OffsetPoint(Group group, PathI path, int j, ref int k)
    {
      if (path[j].Equals(path[k])) { k = j; return; }

      // Let A = change in angle where edges join
      // A == 0: ie no change in angle (flat join)
      // A == PI: edges 'spike'
      // sin(A) < 0: right turning
      // cos(A) < 0: change in angle is more than 90 degree
      float sinA = InternalClipper.CrossProduct(_normals[j], _normals[k]);
      float cosA = math.dot(_normals[j], _normals[k]);
      sinA = math.saturate(sinA);

      if (DeltaCallback != null)
      { 
        _groupDelta = DeltaCallback(path, _normals, j, k);
        if (group.pathsReversed) _groupDelta = -_groupDelta;
      }
      if (Math.Abs(_groupDelta) < Tolerance)
      {
        pathOut.Add(path[j]);
        return;
      }

      if (cosA > -0.999 && (sinA * _groupDelta < 0)) // test for concavity first (#593)
      {
        // is concave
        // by far the simplest way to construct concave joins, especially those joining very 
        // short segments, is to insert 3 points that produce negative regions. These regions 
        // will be removed later by the finishing union operation. This is also the best way 
        // to ensure that path reversals (ie over-spaced paths) are removed.
        pathOut.Add(GetPerpendic(path[j], _normals[k]));
        pathOut.Add(path[j]); // (#405, #873, #916)
        pathOut.Add(GetPerpendic(path[j], _normals[j]));
      }
      else if ((cosA > 0.999) && (_joinType != JoinType.Round))
      {
        // almost straight - less than 2.5 degree (#424, #482, #526 & #724) 
        DoMiter(path, j, k, cosA);
      }
      else switch (_joinType)
      {
        // miter unless the angle is sufficiently acute to exceed ML
        case JoinType.Miter when cosA > _mitLimSqr - 1:
          DoMiter(path, j, k, cosA);
          break;
        case JoinType.Miter:
          DoSquare(path, j, k);
          break;
        case JoinType.Round:
          DoRound(path, j, k, math.atan2(sinA, cosA));
          break;
        case JoinType.Bevel:
          DoBevel(path, j, k);
          break;
        default:
          DoSquare(path, j, k);
          break;
      }

      k = j;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void OffsetPolygon(Group group, PathI path)
    {
      pathOut = new PathI();
      int cnt = path.Count, prev = cnt - 1;
      for (int i = 0; i < cnt; i++)
        OffsetPoint(group, path, i, ref prev);
      _solution.Add(pathOut);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void OffsetOpenJoined(Group group, PathI path)
    {
      OffsetPolygon(group, path);
      path = Clipper.ReversePath(path);
      BuildNormals(path);
      OffsetPolygon(group, path);
    }

    private void OffsetOpenPath(Group group, PathI path)
    {
      pathOut = new PathI();
      int highI = path.Count - 1;

      if (DeltaCallback != null) 
        _groupDelta = DeltaCallback(path, _normals, 0, 0);

      // do the line start cap
      if (Math.Abs(_groupDelta) < Tolerance)
        pathOut.Add(path[0]);
      else
        switch (_endType)
        {
          case EndType.Butt:
            DoBevel(path, 0, 0);
            break;
          case EndType.Round:
            DoRound(path, 0, 0, math.PI);
            break;
          default:
            DoSquare(path, 0, 0);
            break;
        }

      // offset the left side going forward
      for (int i = 1, k = 0; i < highI; i++)
        OffsetPoint(group, path, i, ref k);

      // reverse normals ...
      for (int i = highI; i > 0; i--)
        _normals[i] = new float2(-_normals[i - 1].x, -_normals[i - 1].y);
      _normals[0] = _normals[highI];

      if (DeltaCallback != null)
        _groupDelta = DeltaCallback(path, _normals, highI, highI);
      // do the line end cap
      if (Math.Abs(_groupDelta) < Tolerance)
        pathOut.Add(path[highI]);
      else
        switch (_endType)
        {
          case EndType.Butt:
            DoBevel(path, highI, highI);
            break;
          case EndType.Round:
            DoRound(path, highI, highI, math.PI);
            break;
          default:
            DoSquare(path, highI, highI);
            break;
        }

      // offset the left side going back
      for (int i = highI -1, k = highI; i > 0; i--)
        OffsetPoint(group, path, i, ref k);

      _solution.Add(pathOut);
    }

    private void DoGroupOffset(Group group)
    {
      if (group.endType == EndType.Polygon)
      {
        // a straight path (2 points) can now also be 'polygon' offset 
        // where the ends will be treated as (180 deg.) joins
        if (group.lowestPathIdx < 0) _delta = Math.Abs(_delta);
        _groupDelta = (group.pathsReversed) ? -_delta : _delta;
      }
      else
        _groupDelta = math.abs(_delta);

      float absDelta = math.abs(_groupDelta);

      _joinType = group.joinType;
      _endType = group.endType;

      if (group.joinType == JoinType.Round || group.endType == EndType.Round)
      {
        float arcTol = ArcTolerance > 0.01f ? ArcTolerance : absDelta * arc_const;
        float stepsPer360 = math.PI / math.acos(1 - arcTol / absDelta);
        math.sincos(math.TAU / stepsPer360, out _stepSin, out _stepCos);

        if (_groupDelta < 0.0) _stepSin = -_stepSin;
        _stepsPerRad = stepsPer360 / math.TAU;
      }

      // iterate slices in group's inPaths
      for (int spi = 0; spi < group.inPaths.SlicesCount; spi++)
      {
        var slice = group.inPaths[spi];
        PathI p = new PathI(slice.Length);
        for (int i = 0; i < slice.Length; i++) p.Add(slice[i]);

        pathOut = new PathI();
        int cnt = p.Count;

        switch (cnt)
        {
          case 1:
          {
            int2 pt = p[0];

            if (DeltaCallback != null)
            {
              _groupDelta = DeltaCallback(p, _normals, 0, 0);
              if (group.pathsReversed) _groupDelta = -_groupDelta;
              absDelta = Math.Abs(_groupDelta);
            }

            // single vertex so build a circle or square ...
            if (group.endType == EndType.Round)
            {
              int steps = (int) Math.Ceiling(_stepsPerRad * 2 * Math.PI);
              pathOut = Clipper.Ellipse(pt, (int2)absDelta, steps);
            }
            else
            {
              int d = (int) Math.Ceiling(_groupDelta);
              int4 r = new int4(pt.x - d, pt.y - d, pt.x + d, pt.y + d);
              pathOut = r.AsPath();
            }
            _solution.Add(pathOut);
            continue; // end of offsetting a single point 
          }
          case 2 when group.endType == EndType.Joined:
            _endType = (group.joinType == JoinType.Round) ?
              EndType.Round :
              EndType.Square;
            break;
        }


        BuildNormals(p);
        switch (_endType)
        {
          case EndType.Polygon:
            OffsetPolygon(group, p);
            break;
          case EndType.Joined:
            OffsetOpenJoined(group, p);
            break;
          default:
            OffsetOpenPath(group, p);
            break;
        }
      }
    }
  }

} // namespace