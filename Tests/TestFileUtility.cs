namespace Clipper2Lib.UnitTests
{
	public static class TestFileUtility
	{
		private const string PackageName = "com.longbombus.clipper-bursted";
		private static string filePath;

		public static string GetFullPath(string filename)
		{
			if (filePath == null)
			{
				var packageInfo = UnityEditor.PackageManager.PackageInfo.FindForPackageName(PackageName);
				filePath = packageInfo.assetPath;
			}

			return System.IO.Path.Combine(filePath, filename);
		}
	}
}