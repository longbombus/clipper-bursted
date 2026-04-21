using System;
using NUnit.Framework;

namespace Clipper.Tests
{
  public class TestLines
  {
    [Test]
    public void TestOpenPaths()
    {
      for (int i = 0; i <= 16; i++)
      {
        ClipperI c64 = new();
        PathsI subj = new(), subj_open = new(), clip = new();
        PathsI solution = new(), solution_open = new();

        Assert.IsTrue(
          ClipperFileIO.LoadTestNum(
            "Tests/Lines.txt",
            i, subj, subj_open, clip,
            out var clipType, out var fillrule, out long area, out int count, out _
          ),
          "Loading test {0} failed.", i
        );

        c64.AddSubject(subj);
        c64.AddOpenSubject(subj_open);
        c64.AddClip(clip);
        c64.Execute(clipType, fillrule, solution, solution_open);

        if (area > 0)
        {
          double area2 = Clipper.Area(solution);
          Assert.AreEqual(area, area2, 1e-3, "Test #{0}", i);
        }

        if (count > 0 && Math.Abs(solution.Count - count) > 0)
        {
          Assert.IsTrue(Math.Abs(solution.Count - count) < 2,
            string.Format("Incorrect count in test {0}", i));
        }

      } //bottom of num loop

    }
  }
}