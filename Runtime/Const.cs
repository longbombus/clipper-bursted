using Unity.Mathematics;

namespace Clipper
{
	public static class Const
	{
		public static readonly int4 InvalidRectI = new int4(int.MaxValue, int.MaxValue, int.MinValue, int.MinValue);
		public static readonly float4 InvalidRectF = new float4(float.MaxValue, float.MaxValue, float.MinValue, float.MinValue);

		public const int MaxCoordI = int.MaxValue / 4;
		public const int MinCoordI = -MaxCoordI;
		public const float MaxCoordF = MaxCoordI;
		public const float MinCoordF = -MaxCoordF;
	}
}