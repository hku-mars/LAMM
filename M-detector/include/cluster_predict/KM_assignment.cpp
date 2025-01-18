#include <cluster_predict/KM_assignment.h>



int KMAlgorithm::maxCompatibilitySum(std::vector<std::vector<int>>& students, std::vector<std::vector<int>>& mentors, \
                                     std::vector<int> &match) 
{
    int n = students.size();
    std::vector<std::vector<int>> c(n, std::vector<int>(n, 0)); // student[i]与teacher[j]的兼容性
    for (int i = 0; i < n; ++i) 
    {
        for (int j = 0; j < n; ++j) 
        {
            int K = students[i].size();
            for (int k = 0; k < K; ++k) 
            {
                c[i][j] += students[i][k] == mentors[j][k];
            }
        }
    }
    return KM(n, c, match);
}

int KMAlgorithm::KM(int n, std::vector<std::vector<int>> &c, std::vector<int> &match) 
{
    // std::vector<int> match(n, -1); // match[i]为teacher[i]匹配的student编号
    std::vector<int> exStudent(n); // student的期望
    std::vector<int> exTeacher(n, 0); // teacher的期望
    for (int i = 0; i < n; ++i) 
    {
        exStudent[i] = *max_element(c[i].begin(), c[i].end());
    }
    // 为每个student匹配teacher
    for (int i = 0; i < n; ++i) 
    {
        while(true) 
        {
            std::vector<bool> visStudent(n, false);
            std::vector<bool> visTeacher(n, false);
            if (dfs(i, n, c, match, visStudent, visTeacher, exStudent, exTeacher)) break;
            // 无法匹配降低期望
            for (int j = 0; j < n; ++j) 
            {
                if (visStudent[j]) exStudent[j]--;
                if (visTeacher[j]) exTeacher[j]++;
            }
        }
    }

    int ans = 0;
    for (int i = 0; i < n; ++i) 
    {
        ans += c[match[i]][i];
    }
    return ans;
}

// 匈牙利算法寻找完美匹配
bool dfs(int i, int n, std::vector<std::vector<int>>& c, std::vector<int>& match, std::vector<bool>& visStudent, \
         std::vector<bool>& visTeacher, std::vector<int>& exStudent, std::vector<int>& exTeacher) 
{
    visStudent[i] = true;
    for (int j = 0; j < n; ++j) 
    {
        if (visTeacher[j]) continue;
        int diff = exStudent[i] + exTeacher[j] - c[i][j];
        if (!diff) 
        {
            visTeacher[j] = true;
            if (match[j] == -1 || dfs(match[j], n, c, match, visStudent, visTeacher, exStudent, exTeacher)) 
            {
                match[j] = i;
                return true;
            }
        }
    }
    return false;
}