using UnityEngine;

public class RobotRange : MonoBehaviour
{
    [Header("Range Visualization")]
    public bool showRange = true;
    [Range(8, 128)] public int rangeSegments = 64;
    public Color rangeColor = new(0f, 0.6f, 1f, 0.85f);

    [Header("Cast Area")]
    public float castRadius = 0.5f;
    public float castDistance = 2f;
    [Range(10, 180)] public float castAngle = 60f;

    private LineRenderer rangeRenderer;
    private Robot robot;
    private Vector3 direction;

    void Start()
    {
        robot = gameObject.GetComponent<Robot>();
        direction = GetDirection();
        if (showRange) EnsureRangeRenderer();
    }

    void Update()
    {
        direction = GetDirection();
        if (showRange)
            EnsureRangeRenderer();
        else if (rangeRenderer != null)
            rangeRenderer.enabled = false;
    }

    private Vector3 GetDirection()
    {
        Vector3 robotPosition = gameObject.transform.position;
        if (robot.pathQueue.Count > 0)
        {
            Vector3 targetPosition = robot.pathQueue.Peek();
            Vector3 dir = (targetPosition - robotPosition).normalized;
            return dir;
        }
        return direction;
    }

    private void EnsureRangeRenderer()
    {
        if (gameObject == null) return;

        if (rangeRenderer == null)
        {
            var go = new GameObject($"Robot{robot.robotId}_Range");
            go.transform.SetParent(gameObject.transform, false);

            rangeRenderer = go.AddComponent<LineRenderer>();
            rangeRenderer.useWorldSpace = false;
            rangeRenderer.loop = true;
            rangeRenderer.widthMultiplier = 0.03f;
            rangeRenderer.material = new Material(Shader.Find("Sprites/Default"));
            rangeRenderer.numCornerVertices = 2;
            rangeRenderer.numCapVertices = 2;
        }

        if (rangeRenderer.transform.parent != gameObject.transform)
            rangeRenderer.transform.SetParent(gameObject.transform, false);

        rangeRenderer.enabled = true;
        rangeRenderer.startColor = rangeColor;
        rangeRenderer.endColor = rangeColor;

        int segments = Mathf.Clamp(rangeSegments, 8, 256);
        float angleRad = Mathf.Deg2Rad * castAngle;
        int arcSegments = segments;
        rangeRenderer.positionCount = arcSegments + 2;

        float halfAngle = angleRad / 2f;

        rangeRenderer.SetPosition(0, Vector3.zero);

        for (int i = 0; i <= arcSegments; i++)
        {
            float t = (float)i / arcSegments;
            float theta = Mathf.Lerp(-halfAngle, halfAngle, t);
            Vector3 dir = Quaternion.Euler(0, 0, Mathf.Rad2Deg * theta) * direction;
            Vector3 arcPoint = dir * castDistance + dir.normalized * castRadius;
            rangeRenderer.SetPosition(i + 1, arcPoint);
        }
    }
}
