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
using Unity.Collections;
using Unity.Mathematics;

namespace Clipper
{
  public static class Minkowski
  {
    private static void MinkowskiInternal(NativeArray<int2> pattern, NativeArray<int2> path, NativeSlicedList<int2> solution, bool isSum, bool isClosed)
    {
      int delta = isClosed ? 0 : 1;
      int patLen = pattern.Length, pathLen = path.Length;
      PathsI tmp = new PathsI(pathLen);

      foreach (int2 pathPt in path)
      {
        PathI path2 = new PathI(patLen);
        if (isSum)
        {
          foreach (int2 basePt in pattern)
            path2.Add(pathPt + basePt);
        }
        else
        {
          foreach (int2 basePt in pattern)
            path2.Add(pathPt - basePt);
        }
        tmp.Add(path2);
      }

      solution.EnsureItemsCapacity((pathLen - delta) * patLen);
      int g = isClosed ? pathLen - 1 : 0;

      int h = patLen - 1;
      for (int i = delta; i < pathLen; i++)
      {
        for (int j = 0; j < patLen; j++)
        {
          solution.FinishSlice();
          var quad = solution.AddLastSliceItems(4);
          quad[0] = tmp[g][h];
          quad[1] = tmp[i][h];
          quad[2] = tmp[i][j];
          quad[3] = tmp[g][j];

          if (!Clipper.IsPositive(quad))
            quad.Reverse();

          h = j;
        }
        g = i;
      }
    }

    public static void Sum(NativeArray<int2> pattern, NativeArray<int2> path, NativeSlicedList<int2> solution, bool isClosed)
      => Apply(pattern, path, solution, isClosed, true);

    public static void Sum(NativeArray<float2> pattern, NativeArray<float2> path, NativeSlicedList<float2> solution, bool isClosed, int decimalOrderPrecision = -2)
      => Apply(pattern, path, solution, isClosed, true, math.exp10(decimalOrderPrecision));

    public static void Diff(NativeArray<int2> pattern, NativeArray<int2> path, NativeSlicedList<int2> solution, bool isClosed)
      => Apply(pattern, path, solution, isClosed, false);

    public static void Diff(NativeArray<float2> pattern, NativeArray<float2> path, NativeSlicedList<float2> solution, bool isClosed, int decimalOrderPrecision = -2)
      => Apply(pattern, path, solution, isClosed, false, math.exp10(decimalOrderPrecision));

    private static void Apply(NativeArray<int2> pattern, NativeArray<int2> path, NativeSlicedList<int2> solution, bool isClosed, bool isSum)
    {
      MinkowskiInternal(pattern, path, solution, isSum, isClosed);
      using var c = new ClipperI();
      c.AddPaths(solution, PathType.Subject);
      solution.Clear();
      c.Execute(ClipType.Union, FillRule.NonZero, solution);
    }

    private static void Apply(NativeArray<float2> pattern, NativeArray<float2> path, NativeSlicedList<float2> solution, bool isClosed, bool isSum, float precision)
    {
      var precisionInv = math.rcp(precision);
      using var patternI = Clipper.ScalePath64(pattern, precisionInv, Allocator.Temp);
      using var pathI = Clipper.ScalePath64(path, precisionInv, Allocator.Temp);
      using var solutionI = new NativeSlicedList<int2>(solution.ItemsCapacity, Allocator.Temp);
      Apply(patternI, pathI, solutionI, isClosed, isSum);
      foreach (var solutionPathI in solutionI)
        solution.AddRange(solutionPathI, math.rcp(precision));
    }
  }

} // namespace