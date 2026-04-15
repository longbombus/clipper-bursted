using NUnit.Framework;
using Unity.Collections;

namespace Clipper.Tests
{
  public class TestOffsets
  {

    [Test]
    public void TestOffsetEmpty()
    {
      PathsI solution = new();

      ClipperOffset offset = new ClipperOffset(Allocator.Persistent, Allocator.Temp);
      offset.Execute(10, solution);
    }
  }
}