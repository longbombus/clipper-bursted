using Unity.Mathematics;

namespace Clipper
{
	public static class Const
	{
		public const float PrecisionMin = math.EPSILON;
		public const float PrecisionMax = 1f / math.EPSILON;

		public const int MaxCoordI = int.MaxValue / 4;
		public const int MinCoordI = -MaxCoordI;
		public const float MaxCoordF = MaxCoordI;
		public const float MinCoordF = -MaxCoordF;
	}
}