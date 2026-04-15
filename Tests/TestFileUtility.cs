namespace Clipper.Tests
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
				if (packageInfo == null)
					filePath = "Assets/clipper-bursted";
				else
					filePath = packageInfo.assetPath;
			}

			return System.IO.Path.Combine(filePath, filename);
		}
	}
}