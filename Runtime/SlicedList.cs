using Unity.Collections;
using Unity.Jobs;

namespace Clipper
{
	public struct SlicedList<T> : INativeDisposable
		where T : unmanaged
	{
		private NativeList<T> list;
		private NativeList<int> slices;

		public SlicedList(Allocator allocator)
		{
			list = new NativeList<T>(allocator);
			slices = new NativeList<int>(allocator);
		}

		public void Dispose()
		{
			list.Dispose();
			slices.Dispose();
		}

		public JobHandle Dispose(JobHandle inputDeps)
			=> JobHandle.CombineDependencies(list.Dispose(inputDeps), slices.Dispose(inputDeps));

		public void AddItem(T item)
			=> list.Add(item);

		public void AddSlice()
			=> slices.Add(list.Length);

		public int SliceCount => slices.Length;

		public NativeSlice<T> GetSlice(int index)
		{
			int sliceBegin = index == 0 ? 0 : slices[index - 1];
			int sliceEnd = index < slices.Length ? slices[index] : list.Length;
			return list.AsArray().Slice(sliceBegin, sliceEnd - sliceBegin);
		}

		public NativeArray<T> AsArray() => list.AsArray();
	}
}