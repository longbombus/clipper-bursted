using Clipper;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Clipper.Benchmark
{
	public class ClipperBenchmark : MonoBehaviour
	{
		[SerializeField, Min(0)] private int circlesCount;
		[SerializeField, Min(3)] private int circleSegmentsCount = 24;
		[SerializeField] private ClipType clipType;
		[SerializeField] private FillRule fillRule;

		private void Update()
		{
			var clipper = new ClipperD(2);

			var anglePerCircle = math.TAU / circlesCount;

			for (int i = 0; i < circlesCount; i++)
			{
				float circleTime = Time.unscaledTime + i * anglePerCircle;
				float2 d;
				math.sincos(anglePerCircle * i, out d.x, out d.y);
				math.sincos(circleTime, out var circleOffset, out var circleRadius);

				circleOffset = circleOffset * .5f + .5f;
				circleRadius = circleRadius * .4f + .5f;

				var circle = Clipper.Ellipse(d * circleOffset, circleRadius, circleSegmentsCount);

				var pathType = (i & 1) == 0 ? PathType.Subject : PathType.Clip;
				clipper.AddPath(circle, pathType);
			}

			var solution = new PathsF();
			clipper.Execute(clipType, fillRule, solution);

			foreach (var path in solution)
			{
				var prevPoint = path[^1];
				for (var i = 0; i < path.Length; i++)
				{
					var currPoint = path[i];
					Debug.DrawLine(new Vector3(prevPoint.x, prevPoint.y), new Vector3(currPoint.x, currPoint.y), Color.red);
					prevPoint = currPoint;
				}
			}
		}
	}
}