using NUnit.Framework;

namespace Clipper.Tests
{
  public class TestOffsets
  {

    [Test]
    public void TestOffsetEmpty()
    {
      PathsI solution = new();

      ClipperOffset offset = new ClipperOffset();
      offset.Execute(10, solution);
    }
  }
}