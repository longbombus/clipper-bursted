using NUnit.Framework;

namespace Clipper2Lib.UnitTests
{
  public class TestOffsets
  {

    [Test]
    public void TestOffsetEmpty()
    {
      Paths64 solution = new();

      ClipperOffset offset = new ClipperOffset();
      offset.Execute(10, solution);
    }
  }
}