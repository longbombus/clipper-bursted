using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;

namespace Clipper
{
	[BurstCompile]
	public static class NativeUtility
	{
		[BurstCompile]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Reverse<T>(this NativeArray<T> seq)
			where T : struct
		{
			int lengthHalf = seq.Length / 2;
			int lastIndex = seq.Length - 1;
			for (int i = 0; i < lengthHalf; ++i)
			{
				int j = lastIndex - i;
				(seq[i], seq[j]) = (seq[j], seq[i]);
			}
		}
	}
}