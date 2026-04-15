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

namespace Clipper
{
  public static class Minkowski
  {
    private static PathsI MinkowskiInternal(PathI pattern, PathI path, bool isSum, bool isClosed)
    {
      int delta = isClosed ? 0 : 1;
      int patLen = pattern.Count, pathLen = path.Count;
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

      PathsI result = new PathsI((pathLen - delta) * patLen);
      int g = isClosed ? pathLen - 1 : 0;

      int h = patLen - 1;
      for (int i = delta; i < pathLen; i++)
      {
        for (int j = 0; j < patLen; j++)
        {
          PathI quad = new PathI(4)
          {
            tmp[g][h], tmp[i][h], tmp[i][j], tmp[g][j]
          };
          if (!Clipper.IsPositive(quad))
            result.Add(Clipper.ReversePath(quad));
          else
            result.Add(quad);
          h = j;
        }
        g = i;
      }
      return result;
    }

    public static PathsI Sum(PathI pattern, PathI path, bool isClosed)
    {
      return Clipper.Union(MinkowskiInternal(pattern, path, true, isClosed), FillRule.NonZero);
    }

    public static PathsF Sum(PathF pattern, PathF path, bool isClosed, int decimalPlaces = 2)
    {
      float scale = math.exp10(decimalPlaces);
      PathsI tmp = Clipper.Union(MinkowskiInternal(Clipper.ScalePath64(pattern, scale),
        Clipper.ScalePath64(path, scale), true, isClosed), FillRule.NonZero);
      return Clipper.ScalePathsD(tmp, 1 / scale);
    }

    public static PathsI Diff(PathI pattern, PathI path, bool isClosed)
    {
      return Clipper.Union(MinkowskiInternal(pattern, path, false, isClosed), FillRule.NonZero);
    }

    public static PathsF Diff(PathF pattern, PathF path, bool isClosed, int decimalPlaces = 2)
    {
      float scale = math.exp10(decimalPlaces);
      PathsI tmp = Clipper.Union(MinkowskiInternal(Clipper.ScalePath64(pattern, scale),
        Clipper.ScalePath64(path, scale), false, isClosed), FillRule.NonZero);
      return Clipper.ScalePathsD(tmp, 1 / scale);
    }

  }

} // namespace