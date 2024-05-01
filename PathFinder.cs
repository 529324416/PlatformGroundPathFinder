using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;



/// <summary>坐标</summary>
public struct Vector2Int
{
    public int x;
    public int y;
    public Vector2Int(int x, int y)
    {
        this.x = x;
        this.y = y;
    }
    public static bool operator==(Vector2Int a, Vector2Int b) => 
        a.x == b.x && a.y == b.y;

    public static bool operator!=(Vector2Int a, Vector2Int b) =>
        a.x != b.x || a.y != b.y;

    public override int GetHashCode() => x ^ y;
    public override bool Equals([NotNullWhen(true)] object? obj)
    {
        if(obj is null)return false;
        if(obj is Vector2Int v)
            return v == this;
        return false;
    }
}

/// <summary>
/// A*算法框架
/// </summary>
public abstract class AStarBase
{

    /// <summary>节点</summary>
    protected struct Node
    {
        public readonly Vector2Int pos;
        public readonly int dstFromStart;
        public readonly int dstTotal;

        public Node(Vector2Int pos, int dstFromStart, int dstToEnd)
        {
            this.pos = pos;
            this.dstFromStart = dstFromStart;
            dstTotal = dstToEnd + dstFromStart;
        }
    }

    /// <summary>优先级队列</summary>
    protected class PriorityQueue
    {
        private readonly List<Node> values = new();
        public int Count => values.Count;

        /// <summary>clear current queue</summary>
        public void Clear() => 
            values.Clear();

        /// <summary>put a node into queue</summary>
        public void Put(Node value)
        {
            values.Add(value);
            int i = Count - 1;
            int idx = i / 2;
            while (idx > 0 && values[i].dstTotal < values[idx].dstTotal)
            {
                Swap(i, idx);
                i = idx;
                idx = i / 2;
            }
        }

        /// <summary>get the best node from queue without any safe check</summary>
        public Node Next()
        {
            var node = values[0];
            values[0] = values[Count - 1];
            values.RemoveAt(Count - 1);
            Heapify(Count - 1, 0);
            return node;
        }

        /// <summary>get the best node from queue</summary>
        public bool TryGet(out Node node)
        {
            if (Count == 0) 
            {
                node = default;
                return false;
            }
            node = values[0];
            values[0] = values[Count - 1];
            values.RemoveAt(Count - 1);
            Heapify(Count - 1, 0);
            return true;
        }

        /// <summary>heapify</summary>
        void Heapify(int count, int i)
        {
            while (true)
            {
                int maxPos = i;
                int di = i * 2;
                int di1 = di + 1;
                if (di <= count && values[i].dstTotal > values[di].dstTotal) maxPos = di;
                if (di1 <= count && values[maxPos].dstTotal > values[di1].dstTotal) maxPos = di1;
                if (maxPos == i) break;
                Swap(i, maxPos);
                i = maxPos;
            }
        }
        /// <summary>swap index a and index b</summary>
        void Swap(int a, int b) =>
            (values[b], values[a]) = (values[a], values[b]);
    }
    
    readonly Dictionary<Vector2Int, Node> parentMap = new();
    protected readonly PriorityQueue unexploredSet = new();
    protected readonly Dictionary<Vector2Int, int> costsFromStart = new();
    
    // input
    protected Vector2Int from;
    protected Vector2Int to;

    /// <summary>
    /// 根据查询结果构建路径
    /// </summary>
    Vector2Int[] GenPath()
    {
        var path = new List<Vector2Int>();
        while (to != from)
        {
            path.Add(to);
            to = parentMap[to].pos;
        }
        path.Add(from);
        path.Reverse();
        return path.ToArray();
    }


    protected Vector2Int[] FindPath(Vector2Int from, Vector2Int to)
    {
        if (from == to) return new[] { from };
        
        unexploredSet.Clear();
        parentMap.Clear();
        costsFromStart.Clear();
        
        this.to = to;
        this.from = from;
        unexploredSet.Put(new Node(from, 0, Distance(from, to)));
        costsFromStart.Add(from, 0);
        return FindPath();
    }
    

    Vector2Int[] FindPath()
    {
        while (unexploredSet.Count > 0)
        {
            var node = unexploredSet.Next();
            if (node.pos == to)
                return GenPath();

            int currentCost = node.dstFromStart + 1;
            foreach (var pos in GetNeighbours(node.pos))
            {
                if (costsFromStart.TryGetValue(pos, out int oldGCost) && currentCost >= oldGCost) 
                    continue;
                
                costsFromStart[pos] = currentCost;
                var nextNode = new Node(pos, currentCost, Distance(pos, to));
                parentMap[nextNode.pos] = node;
                unexploredSet.Put(nextNode);
            }
        }

        return null;
    }

    /// <summary>
    /// 获取当前位置的所有可用邻居
    /// </summary>
    /// <param name="pos"></param>
    /// <returns></returns>
    protected abstract Vector2Int[] GetNeighbours(Vector2Int pos);

    /// <summary>
    /// 衡量两个位置之间的距离(默认采用曼哈顿距离)
    /// </summary>
    /// <param name="from"></param>
    /// <param name="to"></param>
    /// <returns></returns>
    protected virtual int Distance(Vector2Int from, Vector2Int to)
    {
        return Math.Abs(from.x - to.x) + Math.Abs(from.y - to.y);
    }
}

public class GameMap
{
    public readonly int w;
    public readonly int h;
    public readonly int[,] map;

    public IEnumerable<Vector2Int> AllGroundPositions{
        get{
            for(int y = 0; y < h; y ++)
                for(int x = 0; x < w; x ++)
                    if(map[x, y] == 2)
                        yield return new Vector2Int(x, y);
        }
    }

    public GameMap(int w, int h, Vector2Int[] obstacles)
    {
        this.w = w;
        this.h = h;
        map = new int[w, h];
        foreach (var pt in obstacles)
            map[pt.x, pt.y] = 1;

        foreach (var pt in GroundSearcher.SearchGround(obstacles, new Vector2Int(w, h), true))
            map[pt.x, pt.y] = 2;
    }


    public bool IsObstacle(int x, int y)
    {
        if(x < 0 || x >= w || y < 0 || y >= h)
            return true;
        return map[x, y] == 1;
    }
    public bool IsGround(int x, int y)
    {
        if(x < 0 || x >= w || y < 0 || y >= h)
            return false;
        return map[x, y] == 2;
    }
}

/// <summary>
/// 地面搜索器
/// </summary>
public static class GroundSearcher
{    
    /// <summary>
    /// 通过边缘检测算法搜索地面，该算法将矩阵视为一个函数，求函数的差分导数以获取地面边缘
    /// </summary>
    /// <param name="obstacles">障碍物坐标</param>
    /// <param name="size">整个地图大小</param>
    /// <param name="handleFloorLine">是否将第一行视为地面</param>
    public static Vector2Int[] SearchGround(Vector2Int[] obstacles, Vector2Int size, bool handleFloorLine = false)
    {
        if (obstacles == null || obstacles.Length == 0)
            return Array.Empty<Vector2Int>();
        
        // 生成原始网格
        int[,] grid = new int[size.x, size.y];
        foreach (var pos in obstacles)
        {
            grid[pos.x, pos.y] = 1;
        }

        // 进行异或运算求地面位置
        int[,] groundMap = new int[size.x, size.y];
        for(int y = 1; y < size.y; y ++)
        for(int x = 0; x < size.x; x ++)
            groundMap[x, y] = grid[x, y - 1] - grid[x, y] > 0 ? 1 : 0;

        if(handleFloorLine)
            for(int x = 0; x < size.x; x ++)
                groundMap[x, 0] = 1 - grid[x, 0] > 0 ? 1 : 0;

        // 读取不为0的网格
        List<Vector2Int> groundPositions = new();
        for(int y = 0; y < size.y; y ++)
        for(int x = 0; x < size.x; x ++)
            if(groundMap[x, y] != 0)
                groundPositions.Add(new Vector2Int(x, y));
                
        return groundPositions.ToArray();
    }

    public static Column[][] GetAllColumns(GameMap map)
    {
        var allColumns = new List<Column>[map.w];
        for (int i = 0; i < allColumns.Length; i++)
            allColumns[i] = new List<Column>();

        foreach (var pos in map.AllGroundPositions)
            allColumns[pos.x].Add(GetColumnFromGroundPoint(map, pos));

        return allColumns.Select(x => x.ToArray()).ToArray();
    }

    /// <summary>
    /// 求地表位置的柱块
    /// </summary>
    /// <param name="map"></param>
    /// <param name="pt"></param>
    /// <returns></returns>
    static Column GetColumnFromGroundPoint(GameMap map, Vector2Int pt)
    {
        int vFrom = pt.y + 1;
        for (int y = vFrom; y < map.h; y++)
        {
            var nextPt = new Vector2Int(pt.x, y);
            if (map.IsObstacle(nextPt.x, nextPt.y))
            {
                return new Column(pt, nextPt.y - vFrom + 1);
            }
        }

        return new Column(pt, map.h - vFrom + 1);
    }
}

public readonly struct Column
{
    public readonly Vector2Int pos;
    private readonly int top;
    public readonly int height => top - pos.y + 1;
    public int bottom => pos.y;
    
    public Column(Vector2Int pos, int height)
    {
        this.pos = pos;
        this.top = pos.y + height - 1;
    }

    /// <summary>
    /// 检查给定的沟壑是否包含在当前沟壑中
    /// </summary>
    /// <param name="other"></param>
    /// <returns></returns>
    public bool Contains(Column other) =>
        bottom <= other.bottom && top >= other.top;

    public bool Touch(Column other, int tolerance)
    {
        if (top < other.bottom || other.top < bottom) return false;
        int minTop = Math.Min(top, other.top);
        int maxBottom = Math.Max(bottom, other.bottom);
        return minTop - maxBottom + 1 >= tolerance;
    }
}


public class AStarGround : AStarBase
{
    private readonly GameMap map;
    private readonly int hRange;
    private readonly int vRange;
    private readonly Column[][] allColumns;
    public AStarGround(GameMap map, int hRange = 4, int vRange = 5)
    {
        this.map = map;
        this.hRange = hRange;
        this.vRange = vRange;
        this.allColumns = GroundSearcher.GetAllColumns(map);
    }
    public Vector2Int[] FindPath(Vector2Int from, Vector2Int to)
    {
        if(!map.IsGround(from.x, from.y) || !map.IsGround(to.x, to.y))
            return null;
        return base.FindPath(from, to);
    }

    protected override Vector2Int[] GetNeighbours(Vector2Int pos)
    {
        return GetJumpPoints(pos, hRange, vRange);
    }

    /// <summary>
    /// 获取给定目标位置附近所有的跳跃点
    /// </summary>
    /// <param name="pos">目标位置</param>
    /// <param name="hRange">横向跳跃距离</param>
    /// <param name="vRange">纵向跳跃距离</param>
    /// <param name="jumpTolerance">一个柱块至少提供jumpTolerance个空间位置，
    /// 才会被算法视为可进行穿越，当jumpTolerance为3时，因为下图中1号障碍物的存在，导致x只能跳到y，而不能到达z
    /// 如果移除1号障碍物，则由于y到2号障碍物的距离能够容纳从x位置开始的jumpTolerance，所以怪物可从
    /// x越过y直接跳到z
    /// </param>
    /// <para>[ ][ ][ ][2][ ][ ][ ]</para>
    /// <para>         [1]         </para>
    /// <para>                     </para>        
    /// <para>[ ][ ] x             </para>
    /// <para>[ ][ ][ ] y  z       </para>
    /// <para>[ ][ ][ ][ ][ ][ ]   </para>
    /// 
    /// <param name="touchTolerance">可忍受的天花板高度(即下图中，要从x跳到y的位置，但是会遇到一个天花板，)
    /// 可忍受的高度为2就是，至少需要2个格子的位置才能从x到y，如果只有一个空间，则认为x无法到达y</param>
    ///          [ ][ ][ ]
    ///           t 
    ///        x  t
    /// [ ][ ][ ] y
    /// <returns></returns>
    public Vector2Int[] GetJumpPoints(Vector2Int pos, int hRange, int vRange, int jumpTolerance = 3, int touchTolerance = 2)
    {
        var columnTolerance = GetCurrentColumn(map, pos, vRange);
        var columnContinues = new Column(pos, jumpTolerance);

        var list = new List<Vector2Int>();
        foreach (var g in FindColumns(columnTolerance, columnContinues, hRange, vRange, 1, touchTolerance))
            list.Add(g.pos);
        foreach (var g in FindColumns(columnTolerance, columnContinues, hRange, vRange, -1, touchTolerance))
            list.Add(g.pos);

        return list.ToArray();
    }

    /// <summary>
    /// 搜索所有符合条件的柱块
    /// </summary>
    /// <param name="tolerance">当前柱块的高度</param>
    /// <param name="continues">跨越受限高度</param>
    /// <param name="hRange">最大可跳跃的横向距离</param>
    /// <param name="vRange">最高可跳跃的纵向距离</param>
    /// <param name="dir">测试方向（1或者-1）</param>
    /// <param name="touchTolerance">至少要多少接触面积才算两个柱块Touch</param>
    /// <param name="step">当前递归到的x</param>
    /// <returns></returns>
    IEnumerable<Column> FindColumns(
        Column tolerance, 
        Column continues, 
        int hRange,
        int vRange,
        int dir,
        int touchTolerance = 2,
        int step = 1)
    {
        bool shouldContinue = false;
        foreach (var g in GetColumnsFromX(tolerance.pos.x + step * dir))
        {
            /* 目标柱块必须与初始柱块至少有tolerance长度的接触面积才算可以进行跳跃 */
            if (!g.Touch(tolerance, touchTolerance)) continue;
            
            /* 该部分检测条件可以自由修改 */
            /* 抛弃底部的空间，并要求两者的纵轴差距在vRange以内，以及斜角连线不得超过45° */
            if (g.pos.y != 0  && Math.Abs(g.pos.y - tolerance.pos.y) <= vRange && DiagonalTest(tolerance, g)) 
                yield return g;

            /* 如果目标柱块可以包含初始柱块，则认为可从初始柱块跃过该柱块(即该柱块大部分都是空白空间) */
            if (!shouldContinue && g.Contains(continues)) 
                shouldContinue = true;
        }
        
        if (step < hRange && shouldContinue)
        {
            foreach (var g in FindColumns(tolerance, continues, hRange, vRange, dir, touchTolerance, step + 1))
                yield return g;
        }
    }

    bool DiagonalTest(Column tolerance, Column g)
    {
        /* 要求当对象从一个平台往下跳跃时，落脚点与跳跃起点的连线不得超过45°
            但是如果两个平台高度差只有1则忽略该条件 */
        
        if (tolerance.bottom <= g.bottom) return true;
        int vDst = tolerance.bottom - g.bottom;
        if (vDst < 2) return true;
        int hDst = Math.Abs(tolerance.pos.x - g.pos.x);
        return hDst < vDst;
    }


    Column[] GetColumnsFromX(int x)
    {
        if (x < 0 || x >= allColumns.Length) return Array.Empty<Column>();
        return allColumns[x];
    }


    Column GetCurrentColumn(GameMap gameMap, Vector2Int current, int vRange)
    {
        int vFrom = current.y + 1;
        int vTo = vRange + current.y;
        int height = 1;
        for (int i = vFrom; i <= vTo; i++)
        {
            var pos = new Vector2Int(current.x, i);
            if (gameMap.IsObstacle(pos.x, pos.y))
            {
                return new Column(current, height);
            }

            height++;
        }

        return new Column(current, height);
    }
}