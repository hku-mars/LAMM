#ifndef HUNGARIAN_H
#define HUNGARIAN_H




template <typename T>
class hungarian 
{  // km
public:
    int n;
    std::vector<int> matchx;  // 左集合对应的匹配点
    std::vector<int> matchy;  // 右集合对应的匹配点
    std::vector<int> pre;     // 连接右集合的左点
    std::vector<bool> visx;   // 拜访数组 左
    std::vector<bool> visy;   // 拜访数组 右
    std::vector<T> lx;
    std::vector<T> ly;
    std::vector<std::vector<T> > g;
    std::vector<T> slack;
    T inf;
    T res;
    std::queue<int> q;
    int org_n;
    int org_m;

    hungarian(int _n, int _m) 
    {
        org_n = _n;
        org_m = _m;
        n = std::max(_n, _m);
        inf = std::numeric_limits<T>::max();
        res = 0;
        g = std::vector<std::vector<T> >(n, std::vector<T>(n));
        matchx = std::vector<int>(n, -1);
        matchy = std::vector<int>(n, -1);
        pre = std::vector<int>(n);
        visx = std::vector<bool>(n);
        visy = std::vector<bool>(n);
        lx = std::vector<T>(n, -inf);
        ly = std::vector<T>(n);
        slack = std::vector<T>(n);
    }

    void addEdge(int u, int v, int w) 
    {
        g[u][v] = std::max(w, 0);  // 负值还不如不匹配 因此设为0不影响
    }

    bool check(int v) 
    {
        visy[v] = true;
        if (matchy[v] != -1)
        {
            q.push(matchy[v]);
            visx[matchy[v]] = true;  // in S
            return false;
        }
        // 找到新的未匹配点 更新匹配点 pre 数组记录着"非匹配边"上与之相连的点
        while (v != -1) 
        {
            matchy[v] = pre[v];
            std::swap(v, matchx[pre[v]]);
        }
        return true;
    }

    void bfs(int i) 
    {
        while (!q.empty()) 
        {
            q.pop();
        }
        q.push(i);
        visx[i] = true;
        while (true) 
        {
            while (!q.empty()) 
            {
                int u = q.front();
                q.pop();
                for (int v = 0; v < n; v++) 
                {
                    if (!visy[v]) 
                    {
                        T delta = lx[u] + ly[v] - g[u][v];
                        if (slack[v] >= delta) 
                        {
                            pre[v] = u;
                            if (delta) 
                            {
                                slack[v] = delta;
                            } 
                            else if (check(v)) 
                            {  // delta=0 代表有机会加入相等子图 找增广路
                                                    // 找到就return 重建交错树
                                return;
                            }
                        }
                    }
                }
            }
            // 没有增广路 修改顶标
            T a = inf;
            for (int j = 0; j < n; j++) 
            {
                if (!visy[j]) 
                {
                    a = std::min(a, slack[j]);
                }
            }
            for (int j = 0; j < n; j++) 
            {
                if (visx[j]) 
                {  // S
                    lx[j] -= a;
                }
                if (visy[j]) 
                {  // T
                    ly[j] += a;
                } 
                else 
                {  // T'
                    slack[j] -= a;
                }
            }
            for (int j = 0; j < n; j++) 
            {
                if (!visy[j] && slack[j] == 0 && check(j)) 
                {
                    return;
                }
            }
        }
    }

    void solve() {
        // 初始顶标
        for (int i = 0; i < n; i++) 
        {
            for (int j = 0; j < n; j++) 
            {
                lx[i] = std::max(lx[i], g[i][j]);
            }
        }

        for (int i = 0; i < n; i++) 
        {
            fill(slack.begin(), slack.end(), inf);
            fill(visx.begin(), visx.end(), false);
            fill(visy.begin(), visy.end(), false);
            bfs(i);
        }

        // custom
        for (int i = 0; i < n; i++) 
        {
            if (g[i][matchx[i]] > 0) 
            {
                res += g[i][matchx[i]];
            } 
            else 
            {
                matchx[i] = -1;
            }
        }
        // std::cout << res << "\n";
        // for (int i = 0; i < org_n; i++) 
        // {
        //     std::cout << matchx[i] + 1 << " ";
        // }
        // std::cout << "\n";
    }

    void GetSolution(int len, std::vector<int> &match)
    {
        // std::cout<<"len:"<<len<<"  size:"<<matchx.size()<<std::endl;
        for(int i=0;i<len;i++)
        {
            match.push_back(matchx[i]);
        }
    }
};


#endif