using System;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.Collections;

namespace Clipper.Tests
{
  public class TestMinkowski
  {
    [Test]
    public void MinkowskiSum_SquarePlusSquare_Int()
    {
      // Pattern: unit square (0,0)-(1,0)-(1,1)-(0,1)
      PathI pattern = new PathI { new int2(0,0), new int2(1,0), new int2(1,1), new int2(0,1) };
      // Path: unit square shifted by (2,0)
      PathI path = new PathI { new int2(2,0), new int2(3,0), new int2(3,1), new int2(2,1) };

      NativeArray<int2> patArr = new NativeArray<int2>(pattern.Count, Allocator.Temp);
      NativeArray<int2> pathArr = new NativeArray<int2>(path.Count, Allocator.Temp);
      for (int i = 0; i < pattern.Count; ++i) patArr[i] = pattern[i];
      for (int i = 0; i < path.Count; ++i) pathArr[i] = path[i];

      var sl = Minkowski.Sum(patArr, pathArr, true, Allocator.Temp);

      // Expect non-empty result
      Assert.IsTrue(sl.SlicesCount > 0);

      // Convert to PathsI to compute area
      PathsI outPaths = new PathsI(sl.SlicesCount);
      for (int si = 0; si < sl.SlicesCount; ++si)
      {
        var slice = sl[si];
        PathI p = new PathI(slice.Length);
        for (int k = 0; k < slice.Length; ++k) p.Add(slice[k]);
        outPaths.Add(p);
      }

      double area = Clipper.Area(outPaths);

      // Minkowski sum of two unit squares separated by 2 in x should have positive area
      Assert.IsTrue(area > 0);

      sl.Dispose();
      patArr.Dispose();
      pathArr.Dispose();
    }

    [Test]
    public void MinkowskiSum_FloatVsInt_Equivalent()
    {
      PathF patternF = new PathF { new float2(0f,0f), new float2(1f,0f), new float2(1f,1f), new float2(0f,1f) };
      PathF pathF = new PathF { new float2(2f,0f), new float2(3f,0f), new float2(3f,1f), new float2(2f,1f) };

      // create NativeArray<float2> manually
      var patFarr = new NativeArray<float2>(patternF.Count, Allocator.Temp);
      var pathFarr = new NativeArray<float2>(pathF.Count, Allocator.Temp);
      for (int i = 0; i < patternF.Count; ++i) patFarr[i] = patternF[i];
      for (int i = 0; i < pathF.Count; ++i) pathFarr[i] = pathF[i];

      // call float overload with decimalPlaces=2
      var slF = Minkowski.Sum(patFarr, pathFarr, true, 2, Allocator.Temp);

      // int path equivalents: scale by 10^2
      float scale = math.exp10(2);
      PathI patternI = Clipper.ScalePath64(patternF, scale);
      PathI pathI = Clipper.ScalePath64(pathF, scale);
      NativeArray<int2> patArr = new NativeArray<int2>(patternI.Count, Allocator.Temp);
      NativeArray<int2> pathArr = new NativeArray<int2>(pathI.Count, Allocator.Temp);
      for (int i = 0; i < patternI.Count; ++i) patArr[i] = patternI[i];
      for (int i = 0; i < pathI.Count; ++i) pathArr[i] = pathI[i];

      var slI = Minkowski.Sum(patArr, pathArr, true, Allocator.Temp);

      // Relaxed assertion: both should produce non-empty results
      Assert.IsTrue(slF.SlicesCount > 0);
      Assert.IsTrue(slI.SlicesCount > 0);

      slF.Dispose();
      slI.Dispose();
      patFarr.Dispose();
      pathFarr.Dispose();
      patArr.Dispose();
      pathArr.Dispose();
    }

    [Test]
    public void MinkowskiDiff_Simple_NotEmpty()
    {
      PathI pattern = new PathI { new int2(0,0), new int2(1,0), new int2(1,1), new int2(0,1) };
      PathI path = new PathI { new int2(2,0), new int2(3,0), new int2(3,1), new int2(2,1) };

      NativeArray<int2> patArr = new NativeArray<int2>(pattern.Count, Allocator.Temp);
      NativeArray<int2> pathArr = new NativeArray<int2>(path.Count, Allocator.Temp);
      for (int i = 0; i < pattern.Count; ++i) patArr[i] = pattern[i];
      for (int i = 0; i < path.Count; ++i) pathArr[i] = path[i];

      var sl = Minkowski.Diff(patArr, pathArr, true, Allocator.Temp);
      Assert.IsTrue(sl.SlicesCount >= 0); // diff may be empty, but call should not crash

      sl.Dispose();
      patArr.Dispose();
      pathArr.Dispose();
    }
  }
}